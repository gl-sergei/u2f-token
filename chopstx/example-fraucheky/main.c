#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <chopstx.h>

#include "config.h"
#include "usb_lld.h"

#define NUM_INTERFACES 1
#define FEATURE_BUS_POWERED	0x80

static chopstx_mutex_t usb_mtx;
static chopstx_cond_t usb_cnd;
static uint32_t bDeviceState = UNCONNECTED; /* USB device status */

extern void EP6_IN_Callback (uint16_t len);
extern void EP6_OUT_Callback (uint16_t len);

#define MSC_MASS_STORAGE_RESET_COMMAND 0xFF
extern int fraucheky_enabled (void);
extern void fraucheky_main (void);

extern void fraucheky_setup_endpoints_for_interface (struct usb_dev *dev, int stop);
extern int fraucheky_setup (struct usb_dev *dev);
extern int fraucheky_get_descriptor (struct usb_dev *dev);

static void
setup_endpoints_for_interface (struct usb_dev *dev, uint16_t interface, int stop)
{
  if (interface == 0)
    fraucheky_setup_endpoints_for_interface (dev, stop);
}

static void
usb_device_reset (struct usb_dev *dev)
{
  int i;

  usb_lld_reset (dev, FEATURE_BUS_POWERED);

  /* Initialize Endpoint 0.  */
  usb_lld_setup_endp (dev, ENDP0, 1, 1);

  /* Stop the interface */
  for (i = 0; i < NUM_INTERFACES; i++)
    setup_endpoints_for_interface (dev, i, 1);

  /* Notify upper layer.  */
  chopstx_mutex_lock (&usb_mtx);
  bDeviceState = ATTACHED;
  chopstx_cond_signal (&usb_cnd);
  chopstx_mutex_unlock (&usb_mtx);
}

static void
usb_ctrl_write_finish (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t type_rcp = arg->type & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT) && arg->index == 0
	   && USB_SETUP_SET (arg->type))
    {
      if (arg->request == MSC_MASS_STORAGE_RESET_COMMAND)
	fraucheky_setup_endpoints_for_interface (dev, 0);
    }
}

static int
usb_setup (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t type_rcp = arg->type & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT)
      && arg->index == 0)
    return fraucheky_setup (dev);

  return -1;
}

static int
usb_set_configuration (struct usb_dev *dev)
{
  int i;
  uint8_t current_conf;

  current_conf = usb_lld_current_configuration (dev);
  if (current_conf == 0)
    {
      if (dev->dev_req.value != 1)
	return -1;

      usb_lld_set_configuration (dev, 1);
      for (i = 0; i < NUM_INTERFACES; i++)
	setup_endpoints_for_interface (dev, i, 0);
      chopstx_mutex_lock (&usb_mtx);
      bDeviceState = CONFIGURED;
      chopstx_mutex_unlock (&usb_mtx);
    }
  else if (current_conf != dev->dev_req.value)
    {
      if (dev->dev_req.value != 0)
	return -1;

      usb_lld_set_configuration (dev, 0);
      for (i = 0; i < NUM_INTERFACES; i++)
	setup_endpoints_for_interface (dev, i, 1);
      chopstx_mutex_lock (&usb_mtx);
      bDeviceState = ADDRESSED;
      chopstx_cond_signal (&usb_cnd);
      chopstx_mutex_unlock (&usb_mtx);
    }

  /* Do nothing when current_conf == value */
  return usb_lld_ctrl_ack (dev);
}


static int
usb_set_interface (struct usb_dev *dev)
{
  uint16_t interface = dev->dev_req.index;
  uint16_t alt = dev->dev_req.value;

  if (interface >= NUM_INTERFACES)
    return -1;

  if (alt != 0)
    return -1;
  else
    {
      setup_endpoints_for_interface (dev, interface, 0);
      return usb_lld_ctrl_ack (dev);
    }
}

static int
usb_get_interface (struct usb_dev *dev)
{
  const uint8_t zero = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  return usb_lld_ctrl_send (dev, &zero, 1);
}


static int
usb_get_status_interface (struct usb_dev *dev)
{
  const uint16_t status_info = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  return usb_lld_ctrl_send (dev, &status_info, 2);
}

static void usb_tx_done (uint8_t ep_num, uint16_t len);
static void usb_rx_ready (uint8_t ep_num, uint16_t len);


#define INTR_REQ_USB SIGUSR1
#define PRIO_USB 3

static void *
usb_main (void *arg)
{
  chopstx_intr_t interrupt;
  struct usb_dev dev;
  int e;

  (void)arg;
  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  usb_lld_init (&dev, FEATURE_BUS_POWERED);
  goto event_handle;	/* For old SYS < 3.0 */

  while (1)
    {
      chopstx_intr_wait (&interrupt);

      if (interrupt.ready)
	{
	  uint8_t ep_num;

	event_handle:
	  e = usb_lld_event_handler (&dev);
	  ep_num = USB_EVENT_ENDP (e);

	  if (ep_num != 0)
	    {
	      if (USB_EVENT_TXRX (e))
		usb_tx_done (ep_num, USB_EVENT_LEN (e));
	      else
		usb_rx_ready (ep_num, USB_EVENT_LEN (e));
	    }
	  else
	    switch (USB_EVENT_ID (e))
	      {
	      case USB_EVENT_DEVICE_RESET:
		usb_device_reset (&dev);
		continue;

	      case USB_EVENT_DEVICE_ADDRESSED:
		chopstx_mutex_lock (&usb_mtx);
		bDeviceState = ADDRESSED;
		chopstx_cond_signal (&usb_cnd);
		chopstx_mutex_unlock (&usb_mtx);
		continue;

	      case USB_EVENT_GET_DESCRIPTOR:
		if (fraucheky_get_descriptor (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_CONFIGURATION:
		if (usb_set_configuration (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_INTERFACE:
		if (usb_set_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_CTRL_REQUEST:
		/* Device specific device request.  */
		if (usb_setup (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_GET_STATUS_INTERFACE:
		if (usb_get_status_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_GET_INTERFACE:
		if (usb_get_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_FEATURE_DEVICE:
	      case USB_EVENT_SET_FEATURE_ENDPOINT:
	      case USB_EVENT_CLEAR_FEATURE_DEVICE:
	      case USB_EVENT_CLEAR_FEATURE_ENDPOINT:
		usb_lld_ctrl_ack (&dev);
		continue;

	      case USB_EVENT_CTRL_WRITE_FINISH:
		/* Control WRITE transfer finished.  */
		usb_ctrl_write_finish (&dev);
		continue;

	      case USB_EVENT_OK:
	      case USB_EVENT_DEVICE_SUSPEND:
	      default:
		continue;
	      }
	}
    }

  return NULL;
}

static void
usb_tx_done (uint8_t ep_num, uint16_t len)
{
  if (ep_num == ENDP6)
    EP6_IN_Callback (len);
}

static void
usb_rx_ready (uint8_t ep_num, uint16_t len)
{
  if (ep_num == ENDP6)
    EP6_OUT_Callback (len);
}

static char __process3_stack_base__[4096];

#define STACK_ADDR_USB ((uintptr_t)__process3_stack_base__)
#define STACK_SIZE_USB (sizeof __process3_stack_base__)

#ifdef GNU_LINUX_EMULATION
#define main emulated_main
#endif

/*
 * Entry point.
 *
 * NOTE: the main function is already a thread in the system on entry.
 */
int
main (int argc, char **argv)
{
  chopstx_t usb_thd;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&usb_mtx);
  chopstx_cond_init (&usb_cnd);

  bDeviceState = UNCONNECTED;
  usb_thd = chopstx_create (PRIO_USB, STACK_ADDR_USB, STACK_SIZE_USB,
			    usb_main, NULL);
  while (bDeviceState != CONFIGURED)
    chopstx_usec_wait (250*1000);
  fraucheky_main ();
  chopstx_cancel (usb_thd);
  chopstx_join (usb_thd, NULL);
  usb_lld_shutdown ();
  bDeviceState = UNCONNECTED;

  return 0;
}
