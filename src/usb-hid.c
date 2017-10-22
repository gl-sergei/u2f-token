/*
 * usb-hid.c - HID device descriptors and communication
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware for STM32F103 and EFM32HG boards
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <eventflag.h>
#include <string.h>
#include "board.h"
#include "sys.h"
#include "usb_lld.h"

#include "usb-hid.h"


#define ENDP0_RXADDR        (0x040)
#define ENDP0_TXADDR        (0x080)
#define ENDP1_RXADDR        (0x0c0)
#define ENDP1_TXADDR        (0x100)

#define HID_INTERFACE 0

#define USB_HID_REQ_GET_REPORT   1
#define USB_HID_REQ_GET_IDLE     2
#define USB_HID_REQ_GET_PROTOCOL 3
#define USB_HID_REQ_SET_REPORT   9
#define USB_HID_REQ_SET_IDLE     10
#define USB_HID_REQ_SET_PROTOCOL 11

#define USB_DT_HID                      0x21
#define USB_DT_REPORT                   0x22

// Size of HID reports 

#define HID_RPT_SIZE            64      // Default size of raw HID report

static chopstx_intr_t interrupt;

static uint8_t hid_idle_rate;   /* in 4ms */

/* HID report descriptor.  */
#define HID_REPORT_DESC_SIZE (sizeof (hid_report_desc))

static const uint8_t hid_report_desc[] = {
  0x06, 0xd0, 0xf1, /* USAGE_PAGE (FIDO Alliance) */
  0x09, 0x01,       /* USAGE (Keyboard) */

  0xa1, 0x01,       /* COLLECTION (Application) */

  0x09, 0x20,       /*   USAGE (Input report data) */
  0x15, 0x00,       /*   LOGICAL_MINIMUM (0) */
  0x26, 0xff, 0x00, /*   LOGICAL_MAXIMUM (255) */
  0x75, 0x08,       /*   REPORT_SIZE (8) */
  0x95, 0x40,       /*   REPORT_COUNT (64) */
  0x81, 0x02,       /*   INPUT (Data,Var,Abs); Modifier byte */

  0x09, 0x21,       /*   USAGE (Output report data) */
  0x15, 0x00,       /*   LOGICAL_MINIMUM (0) */
  0x26, 0xff, 0x00, /*   LOGICAL_MAXIMUM (255) */
  0x75, 0x08,       /*   REPORT_SIZE (8) */
  0x95, 0x40,       /*   REPORT_COUNT (64) */
  0x91, 0x02,       /*   OUTPUT (Data,Var,Abs); Modifier byte */

  0xc0              /* END_COLLECTION */
};

/* USB Device Descriptor */
static const uint8_t u2f_device_desc[18] = {
  18,   /* bLength */
  DEVICE_DESCRIPTOR,                /* bDescriptorType */
  0x10, 0x01,                        /* bcdUSB = 1.1 */
  0x00,                                /* bDeviceClass (Unknown).          */
  0x00,                                /* bDeviceSubClass.                 */
  0x00,                                /* bDeviceProtocol.                 */
  0x40,                                /* bMaxPacketSize.                  */
  0x83, 0x04, /* idVendor  */
  0xab, 0xcd, /* idProduct */
  0x00, 0x01, /* bcdDevice  */
  1,                                /* iManufacturer.                   */
  2,                                /* iProduct.                        */
  3,                                /* iSerialNumber.                   */
  1                                /* bNumConfigurations.              */
};

#define FEATURE_BUS_POWERED        0x80

/* Configuration Descriptor tree for a HID.*/
static const uint8_t u2f_config_desc[41] = {
  9,
  CONFIG_DESCRIPTOR,                /* bDescriptorType: Configuration */
  /* Configuration Descriptor.*/
  41, 0x00,                        /* wTotalLength.                    */
  0x01,                                /* bNumInterfaces.                  */
  0x01,                                /* bConfigurationValue.             */
  0,                                /* iConfiguration.                  */
  FEATURE_BUS_POWERED,        /* bmAttributes.                    */
  50,                                /* bMaxPower (100mA).               */

  /* Interface Descriptor.*/
  9,               /* bLength: Interface Descriptor size */
  INTERFACE_DESCRIPTOR, /* bDescriptorType: Interface */
  HID_INTERFACE,     /* bInterfaceNumber: Number of Interface */
  0x00,     /* bAlternateSetting: Alternate setting */
  0x02,     /* bNumEndpoints: Two endpoints used */
  0x03,     /* bInterfaceClass: HID */
  0x00,     /* bInterfaceSubClass: no boot */
  0x00,     /* bInterfaceProtocol: 0=none */
  0x04,     /* iInterface */

  /* HID Descriptor.*/
  9,          /* bLength: HID Descriptor size */
  0x21,         /* bDescriptorType: HID */
  0x10, 0x01,   /* bcdHID: HID Class Spec release number */
  0x00,         /* bCountryCode: Hardware target country */
  0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,         /* bDescriptorType */
  HID_REPORT_DESC_SIZE, 0, /* wItemLength: Total length of Report descriptor */

  /*Endpoint IN1 Descriptor*/
  7,                            /* bLength: Endpoint Descriptor size */
  ENDPOINT_DESCRIPTOR,    /* bDescriptorType: Endpoint */
  0x81,       /* bEndpointAddress: (IN1) */
  0x03,       /* bmAttributes: Interrupt */
  0x40, 0x00,     /* wMaxPacketSize: 64 */
  0x05,       /* bInterval (5ms) */

  /*Endpoint OUT1 Descriptor*/
  7,                            /* bLength: Endpoint Descriptor size */
  ENDPOINT_DESCRIPTOR,    /* bDescriptorType: Endpoint */
  0x01,       /* bEndpointAddress: (OUT1) */
  0x03,       /* bmAttributes: Interrupt */
  0x40, 0x00,     /* wMaxPacketSize: 64 */
  0x05,       /* bInterval (5ms) */
};


/*
 * U.S. English language identifier.
 */
static const uint8_t usb_string0[4] = {
  4,                                /* bLength */
  STRING_DESCRIPTOR,
  0x09, 0x04                        /* LangID = 0x0409: US-English */
};

static const uint8_t usb_string1[] = {
  9*2+2,                            /* bLength */
  STRING_DESCRIPTOR,                /* bDescriptorType */
  /* Manufacturer: "Gl.Sergei" */
  'G', 0, 'l', 0, '.', 0, 'S', 0, 'e', 0, 'r', 0, 'g', 0, 'e', 0,
  'i', 0,
};

static const uint8_t usb_string2[] = {
  17*2+2,                           /* bLength */
  STRING_DESCRIPTOR,                /* bDescriptorType */
#if defined(MCU_EFM32HG)
  /* Product name: "U2F-token (EFM32)" */
  'U', 0, '2', 0, 'F', 0, '-', 0, 't', 0, 'o', 0, 'k', 0, 'e', 0,
  'n', 0, ' ', 0, '(', 0, 'E', 0, 'F', 0, 'M', 0, '3', 0, '2', 0,
#else
  /* Product name: "U2F-token (STM32)" */
  'U', 0, '2', 0, 'F', 0, '-', 0, 't', 0, 'o', 0, 'k', 0, 'e', 0,
  'n', 0, ' ', 0, '(', 0, 'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0,
#endif
  ')', 0,
};

/*
 * Serial Number string.
 */
static const uint8_t usb_string3[28] = {
  28,                                    /* bLength */
  STRING_DESCRIPTOR,                    /* bDescriptorType */
  '0', 0,  '.', 0,  '0', 0, '0', 0, /* Version number */
};


#define NUM_INTERFACES 1


static void
usb_device_reset (struct usb_dev *dev)
{
  usb_lld_reset (dev, FEATURE_BUS_POWERED);

  /* Initialize Endpoint 0 */
#if defined(MCU_EFM32HG)
  usb_lld_setup_endp (dev, ENDP0, EP_CONTROL, 1, 1, HID_RPT_SIZE);
#else
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0,
            ENDP0_RXADDR, ENDP0_TXADDR, HID_RPT_SIZE);
#endif
}



static void
usb_ctrl_write_finish (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;

  if (arg->index == HID_INTERFACE && arg->request == USB_HID_REQ_SET_REPORT)
    {
      return;
    }

  /*
   * The transaction was already finished.  So, it is no use to call
   * usb_lld_ctrl_error when the condition does not match.
   */
}



static int
usb_setup (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;

  if (arg->index == HID_INTERFACE)
    {
      switch (arg->request)
        {
        case USB_HID_REQ_GET_IDLE:
          return usb_lld_ctrl_send (dev, &hid_idle_rate, 1);
        case USB_HID_REQ_SET_IDLE:
          hid_idle_rate = arg->value >> 8;
          return usb_lld_ctrl_ack (dev);

        case USB_HID_REQ_GET_REPORT:
          /* Reports should go via interrupt endpoint */
          return -1;

        case USB_HID_REQ_SET_REPORT:
          /* Reports should go via interrupt endpoint */
          return -1;

        case USB_HID_REQ_GET_PROTOCOL:
        case USB_HID_REQ_SET_PROTOCOL:
          /* This driver doesn't support boot protocol.  */
          return -1;

        default:
          return -1;
        }
    }

  return -1;
}

static int
usb_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;
  uint8_t desc_type = (arg->value >> 8);
  uint8_t desc_index = (arg->value & 0xff);

  if (rcp == INTERFACE_RECIPIENT)
    {
      if (arg->index == HID_INTERFACE)
        {
          if (desc_type == USB_DT_HID)
            return usb_lld_ctrl_send (dev, u2f_config_desc + 9 + 9, 9);
          else if (desc_type == USB_DT_REPORT)
            return usb_lld_ctrl_send (dev, hid_report_desc,
                                      HID_REPORT_DESC_SIZE);
        }
    }
  else if (rcp == DEVICE_RECIPIENT)
    {
      if (desc_type == DEVICE_DESCRIPTOR)
        return usb_lld_ctrl_send (dev,
                                  u2f_device_desc, sizeof (u2f_device_desc));
      else if (desc_type == CONFIG_DESCRIPTOR)
        return usb_lld_ctrl_send (dev,
                                  u2f_config_desc, sizeof (u2f_config_desc));
      else if (desc_type == STRING_DESCRIPTOR)
        {
          const uint8_t *str;
          int size;

          switch (desc_index)
            {
            case 0:
              str = usb_string0;
              size = sizeof (usb_string0);
              break;
            case 1:
              str = usb_string1;
              size = sizeof (usb_string1);
              break;
            case 2:
              str = usb_string2;
              size = sizeof (usb_string2);
              break;
            case 3:
              str = usb_string3;
              size = sizeof (usb_string3);
              break;
            default:
              return -1;
            }

          return usb_lld_ctrl_send (dev, str, size);
        }
  }

  return -1;
}

static void
hid_setup_endpoints_for_interface (struct usb_dev *dev,
                                   uint16_t interface, int stop)
{
#if !defined(MCU_EFM32HG)
  (void) dev;
#endif
  if (interface == HID_INTERFACE)
    {
      if (!stop)
#if defined(MCU_EFM32HG)
        usb_lld_setup_endp (dev, ENDP1, EP_INTERRUPT, 1, 1, HID_RPT_SIZE);
#else
        usb_lld_setup_endpoint (ENDP1, EP_INTERRUPT, 0,
                  ENDP1_RXADDR, ENDP1_TXADDR, HID_RPT_SIZE);
#endif
      else
        {
          usb_lld_stall_tx (ENDP1);
          usb_lld_stall_rx (ENDP1);
        }
    }
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
        hid_setup_endpoints_for_interface (dev, i, 0);
    }
  else if (current_conf != dev->dev_req.value)
    {
      if (dev->dev_req.value != 0)
        return -1;

      usb_lld_set_configuration (dev, 0);
      for (i = 0; i < NUM_INTERFACES; i++)
        hid_setup_endpoints_for_interface (dev, i, 1);
    }

  usb_lld_ctrl_ack (dev);
  return 0;
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
      hid_setup_endpoints_for_interface (dev, interface, 0);
      usb_lld_ctrl_ack (dev);
      return 0;
    }
}

static int
usb_get_interface (struct usb_dev *dev)
{
  const uint8_t zero = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  /* We don't have alternate interface, so, always return 0.  */
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

struct usb_hid {
#if defined(MCU_EFM32HG)
  uint8_t tx_buf[HID_RPT_SIZE] __attribute__((aligned(4)));
#else
  uint8_t *tx_buf;
#endif
  uint8_t rx_buf[HID_RPT_SIZE] __attribute__((aligned(4)));
  uint16_t tx_len;
  uint16_t rx_len;
  struct eventflag cmd_ev;
  struct eventflag usb_ev;
  struct eventflag fbk_ev;
};

/* cmd events */
#define EV_TX_NEED         1

/* usb events */
#define EV_RX_DATA_READY   1

/* fbk events */
#define EV_TX_FINISHED     1

/*
 * Rx ready callback
 */

static void
ep1_out_received (struct usb_hid *hid, uint16_t len)
{
  if (len <= HID_RPT_SIZE)
#if defined(MCU_EFM32HG)
    hid->rx_len = len;
#else
    {
      usb_lld_rxcpy (hid->rx_buf, 1 /*ep_num*/, 0 /*offset*/, len);
      hid->rx_len = len;
    }
#endif

  eventflag_signal (&hid->usb_ev, EV_RX_DATA_READY);
}


/*
 * Transmit some data
 */

static void
ep1_transmit (struct usb_hid *hid)
{
#if defined(MCU_EFM32HG)
  usb_lld_tx_enable_buf (ENDP1, hid->tx_buf, hid->tx_len);
#else
  usb_lld_txcpy (hid->tx_buf, 1/*ep_num*/, 0 /*offset*/, hid->tx_len);
  usb_lld_tx_enable (1/*ep_num*/, hid->tx_len);
#endif
}

static void
usb_rx_ready (struct usb_hid *hid, uint8_t ep_num, uint16_t len)
{
  if (ep_num == ENDP1)
    ep1_out_received (hid, len);
}

static void
usb_tx_done (struct usb_hid *hid, uint8_t ep_num, uint16_t len)
{
  (void)hid;
  (void)ep_num;
  (void)len;

  if (ep_num == ENDP1)
    {
      eventflag_signal (&hid->fbk_ev, EV_TX_FINISHED);
    }
}

static void
usb_tx (struct usb_hid *hid, uint8_t ep_num)
{
  if (ep_num == ENDP1)
    ep1_transmit (hid);
}


#if defined(MCU_EFM32HG)
#define INTR_REQ_USB 19
#else
#define INTR_REQ_USB 20
#endif
#define PRIO_HID     8

extern uint8_t __process3_stack_base__[], __process3_stack_size__[];
#define STACK_ADDR_HID ((uint32_t)__process3_stack_base__)
#define STACK_SIZE_HID ((uint32_t)__process3_stack_size__)


static void
poll_tx_intr (uint32_t *timeout, struct eventflag *ev, chopstx_intr_t *intr)
{
  chopstx_poll_cond_t poll_desc;
  struct chx_poll_head *pd_array[2] = {
    (struct chx_poll_head *)intr,
    (struct chx_poll_head *)&poll_desc
  };

  eventflag_prepare_poll (ev, &poll_desc);
  chopstx_poll (timeout, 2, pd_array);
}

static struct usb_dev dev __attribute__((aligned(4)));

static void *
hid_main (void *arg)
{
  uint32_t timeout;
  int e;

  struct usb_hid *usb_hid = (struct usb_hid *) arg;

  memset (usb_hid, 0, sizeof (struct usb_hid));

  eventflag_init (&usb_hid->cmd_ev);
  eventflag_init (&usb_hid->usb_ev);
  eventflag_init (&usb_hid->fbk_ev);

  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  usb_lld_init (&dev, FEATURE_BUS_POWERED);

  while (1)
    {
      eventmask_t m;

      timeout = 1950 * 1000;
      poll_tx_intr (&timeout, &usb_hid->cmd_ev, &interrupt);

      if (interrupt.ready)
        {
          uint8_t ep_num;
          /*
           * When interrupt is detected, call usb_lld_event_handler.
           * The event may be one of following:
           *    (1) Transfer to endpoint (bulk or interrupt)
           *        In this case EP_NUM is encoded in the variable E.
           *    (2) "NONE" event: some trasfer was done, but all was
           *        done by lower layer, no other work is needed in
           *        upper layer.
           *    (3) Device events: Reset or Suspend
           *    (4) Device requests to the endpoint zero.
           *        
           */
          e = usb_lld_event_handler (&dev);
          ep_num = USB_EVENT_ENDP (e);

          if (ep_num != 0)
            {
              if (USB_EVENT_TXRX (e))
                usb_tx_done (usb_hid, ep_num, USB_EVENT_LEN (e));
              else
                usb_rx_ready (usb_hid, ep_num, USB_EVENT_LEN (e));
            }
          else
            switch (USB_EVENT_ID (e))
              {
              case USB_EVENT_DEVICE_RESET:
                usb_device_reset (&dev);
                continue;

              case USB_EVENT_DEVICE_ADDRESSED:
                /* The addres is assigned to the device.  We don't
                 * need to do anything for this actually, but in this
                 * application, we maintain the USB status of the
                 * device.  Usually, just "continue" as EVENT_OK is
                 * OK.
                 */
                continue;

              case USB_EVENT_GET_DESCRIPTOR:
                if (usb_get_descriptor (&dev) < 0)
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

      m = eventflag_get (&usb_hid->cmd_ev);

      if (m == EV_TX_NEED)
        {
          usb_tx (usb_hid, ENDP1);
        }

    }

  return NULL;
}

static struct usb_hid usb_hid __attribute__((aligned(4)));

struct usb_hid *
hid_open (void)
{
  chopstx_create (PRIO_HID, STACK_ADDR_HID, STACK_SIZE_HID, hid_main, &usb_hid);
  return &usb_hid;
}

int
hid_send (struct usb_hid *usb_hid, uint8_t *buf, uint16_t len)
{
  eventmask_t m;

  if (len > HID_RPT_SIZE)
    return -1;

  usb_hid->tx_len = len;
#if defined(MCU_EFM32HG)
  memcpy (usb_hid->tx_buf, buf, len);
#else
  usb_hid->tx_buf = buf;
#endif
  eventflag_signal (&usb_hid->cmd_ev, EV_TX_NEED);

  do
    {
      m = eventflag_wait (&usb_hid->fbk_ev);
    }
  while (m != EV_TX_FINISHED);

  return len;
}

int
hid_recv (struct usb_hid *usb_hid, uint8_t *buf, uint16_t len, uint32_t timeout)
{
  eventmask_t m;

#if defined(MCU_EFM32HG)
  usb_lld_rx_enable_buf (ENDP1, &usb_hid->rx_buf, HID_RPT_SIZE);
#else
  usb_lld_rx_enable (ENDP1);
#endif

  do
    {
      m = eventflag_wait_timeout (&usb_hid->usb_ev, timeout);
    }
  while (m != EV_RX_DATA_READY && m != 0);

  if (m == 0)
    return -1;

  if (usb_hid->rx_len <= len)
    {
      memcpy (buf, usb_hid->rx_buf, usb_hid->rx_len);
      return usb_hid->rx_len;
    }

  return -1;
}
