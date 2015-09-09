#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "usb_lld.h"

extern chopstx_mutex_t usb_mtx;
extern chopstx_cond_t cnd_usb;

#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x80)
#define ENDP1_TXADDR        (0xc0)
#define ENDP2_TXADDR        (0x100)
#define ENDP3_RXADDR        (0x140)

#define USB_CDC_REQ_SET_LINE_CODING             0x20
#define USB_CDC_REQ_GET_LINE_CODING             0x21
#define USB_CDC_REQ_SET_CONTROL_LINE_STATE      0x22
#define USB_CDC_REQ_SEND_BREAK                  0x23

/* USB Device Descriptor */
static const uint8_t vcom_device_desc[18] = {
  18,   /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,	/* bDescriptorType */
  0x10, 0x01,			/* bcdUSB = 1.1 */
  0x02,				/* bDeviceClass (CDC).              */
  0x00,				/* bDeviceSubClass.                 */
  0x00,				/* bDeviceProtocol.                 */
  0x40,				/* bMaxPacketSize.                  */
  0xFF, 0xFF, /* idVendor  */
  0x01, 0x00, /* idProduct */
  0x00, 0x01, /* bcdDevice  */
  1,				/* iManufacturer.                   */
  2,				/* iProduct.                        */
  3,				/* iSerialNumber.                   */
  1				/* bNumConfigurations.              */
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_config_desc[67] = {
  9,
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  /* Configuration Descriptor.*/
  67, 0x00,			/* wTotalLength.                    */
  0x02,				/* bNumInterfaces.                  */
  0x01,				/* bConfigurationValue.             */
  0,				/* iConfiguration.                  */
  0x80,				/* bmAttributes (bus powered).      */
  50,				/* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  9,
  USB_INTERFACE_DESCRIPTOR_TYPE,
  0x00,		   /* bInterfaceNumber.                */
  0x00,		   /* bAlternateSetting.               */
  0x01,		   /* bNumEndpoints.                   */
  0x02,		   /* bInterfaceClass (Communications Interface Class,
		      CDC section 4.2).  */
  0x02,		   /* bInterfaceSubClass (Abstract Control Model, CDC
		      section 4.3).  */
  0x01,		   /* bInterfaceProtocol (AT commands, CDC section
		      4.4).  */
  0,	           /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  5,	      /* bLength.                         */
  0x24,	      /* bDescriptorType (CS_INTERFACE).  */
  0x00,	      /* bDescriptorSubtype (Header Functional Descriptor). */
  0x10, 0x01, /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  5,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x01,         /* bDescriptorSubtype (Call Management Functional
		   Descriptor). */
  0x03,         /* bmCapabilities (D0+D1).          */
  0x01,         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  4,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x02,         /* bDescriptorSubtype (Abstract Control Management
		   Descriptor).  */
  0x02,         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  5,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x06,         /* bDescriptorSubtype (Union Functional
		   Descriptor).  */
  0x00,         /* bMasterInterface (Communication Class
		   Interface).  */
  0x01,         /* bSlaveInterface0 (Data Class Interface).  */
  /* Endpoint 2 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,
  ENDP2|0x80,    /* bEndpointAddress.    */
  0x03,          /* bmAttributes (Interrupt).        */
  0x08, 0x00,	 /* wMaxPacketSize.                  */
  0xFF,		 /* bInterval.                       */
  /* Interface Descriptor.*/
  9,
  USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType: */
  0x01,          /* bInterfaceNumber.                */
  0x00,          /* bAlternateSetting.               */
  0x02,          /* bNumEndpoints.                   */
  0x0A,          /* bInterfaceClass (Data Class Interface, CDC section 4.5). */
  0x00,          /* bInterfaceSubClass (CDC section 4.6). */
  0x00,          /* bInterfaceProtocol (CDC section 4.7). */
  0x00,		 /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: Endpoint */
  ENDP3,    /* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00,				/* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: Endpoint */
  ENDP1|0x80,			/* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00				/* bInterval.                       */
};


/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[4] = {
  4,				/* bLength */
  USB_STRING_DESCRIPTOR_TYPE,
  0x09, 0x04			/* LangID = 0x0409: US-English */
};

static const uint8_t vcom_string1[] = {
  23*2+2,			/* bLength */
  USB_STRING_DESCRIPTOR_TYPE,	/* bDescriptorType */
  /* Manufacturer: "Flying Stone Technology" */
  'F', 0, 'l', 0, 'y', 0, 'i', 0, 'n', 0, 'g', 0, ' ', 0, 'S', 0,
  't', 0, 'o', 0, 'n', 0, 'e', 0, ' ', 0, 'T', 0, 'e', 0, 'c', 0,
  'h', 0, 'n', 0, 'o', 0, 'l', 0, 'o', 0, 'g', 0, 'y', 0, 
};

static const uint8_t vcom_string2[] = {
  14*2+2,			/* bLength */
  USB_STRING_DESCRIPTOR_TYPE,	/* bDescriptorType */
  /* Product name: "Chopstx Sample" */
  'C', 0, 'h', 0, 'o', 0, 'p', 0, 's', 0, 't', 0, 'x', 0, ' ', 0,
  'S', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0,
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[28] = {
  28,				    /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,	    /* bDescriptorType */
  '0', 0,  '.', 0,  '0', 0, '0', 0, /* Version number */
};


#define NUM_INTERFACES 2

uint32_t bDeviceState = UNCONNECTED; /* USB device status */
uint8_t connected;

void
usb_cb_device_reset (void)
{
  /* Set DEVICE as not configured */
  usb_lld_set_configuration (0);

  /* Current Feature initialization */
  usb_lld_set_feature (vcom_config_desc[7]);

  usb_lld_reset ();

  /* Initialize Endpoint 0 */
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0, ENDP0_RXADDR, ENDP0_TXADDR, 64);

  chopstx_mutex_lock (&usb_mtx);
  connected = 0;
  bDeviceState = ATTACHED;
  chopstx_cond_signal (&cnd_usb);
  chopstx_mutex_unlock (&usb_mtx);
}


#define CDC_CTRL_DTR            0x0001

void
usb_cb_ctrl_write_finish (uint8_t req, uint8_t req_no, uint16_t value)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT)
      && USB_SETUP_SET (req) && req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
    {
      /* Open/close the connection.  */
      chopstx_mutex_lock (&usb_mtx);
      connected = ((value & CDC_CTRL_DTR) != 0)? 1 : 0;
      chopstx_cond_signal (&cnd_usb);
      chopstx_mutex_unlock (&usb_mtx);
    }
}

struct line_coding
{
  uint32_t bitrate;
  uint8_t format;
  uint8_t paritytype;
  uint8_t datatype;
}  __attribute__((packed));

static struct line_coding line_coding = {
  115200, /* baud rate: 115200    */
  0x00,   /* stop bits: 1         */
  0x00,   /* parity:    none      */
  0x08    /* bits:      8         */
};


static int
vcom_port_data_setup (uint8_t req, uint8_t req_no, struct control_info *detail)
{
  if (USB_SETUP_GET (req))
    {
      if (req_no == USB_CDC_REQ_GET_LINE_CODING)
	return usb_lld_reply_request (&line_coding, sizeof(line_coding), detail);
    }
  else  /* USB_SETUP_SET (req) */
    {
      if (req_no == USB_CDC_REQ_SET_LINE_CODING
	  && detail->len == sizeof (line_coding))
	{
	  usb_lld_set_data_to_recv (&line_coding, sizeof (line_coding));
	  return USB_SUCCESS;
	}
      else if (req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
	return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

int
usb_cb_setup (uint8_t req, uint8_t req_no, struct control_info *detail)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT) && detail->index == 0)
    return vcom_port_data_setup (req, req_no, detail);

  return USB_UNSUPPORT;
}

int
usb_cb_get_descriptor (uint8_t rcp, uint8_t desc_type, uint8_t desc_index,
		       struct control_info *detail)
{
  if (rcp != DEVICE_RECIPIENT)
    return USB_UNSUPPORT;

  if (desc_type == DEVICE_DESCRIPTOR)
    return usb_lld_reply_request (vcom_device_desc, sizeof (vcom_device_desc),
				  detail);
  else if (desc_type == CONFIG_DESCRIPTOR)
    return usb_lld_reply_request (vcom_config_desc, sizeof (vcom_config_desc),
				  detail);
  else if (desc_type == STRING_DESCRIPTOR)
    {
      const uint8_t *str;
      int size;

      switch (desc_index)
	{
	case 0:
	  str = vcom_string0;
	  size = sizeof (vcom_string0);
	  break;
	case 1:
	  str = vcom_string1;
	  size = sizeof (vcom_string1);
	  break;
	case 2:
	  str = vcom_string2;
	  size = sizeof (vcom_string2);
	  break;
	case 3:
	  str = vcom_string3;
	  size = sizeof (vcom_string3);
	  break;
	default:
	  return USB_UNSUPPORT;
	}

      return usb_lld_reply_request (str, size, detail);
    }

  return USB_UNSUPPORT;
}

static void
vcom_setup_endpoints_for_interface (uint16_t interface, int stop)
{
  if (interface == 0)
    {
      if (!stop)
	usb_lld_setup_endpoint (ENDP2, EP_INTERRUPT, 0, 0, ENDP2_TXADDR, 0);
      else
	usb_lld_stall_tx (ENDP2);
    }
  else if (interface == 1)
    {
      if (!stop)
	{
	  usb_lld_setup_endpoint (ENDP1, EP_BULK, 0, 0, ENDP1_TXADDR, 0);
	  usb_lld_setup_endpoint (ENDP3, EP_BULK, 0, ENDP3_RXADDR, 0, 64);
	}
      else
	{
	  usb_lld_stall_tx (ENDP1);
	  usb_lld_stall_rx (ENDP3);
	}
    }
}

int usb_cb_handle_event (uint8_t event_type, uint16_t value)
{
  int i;
  uint8_t current_conf;

  switch (event_type)
    {
    case USB_EVENT_ADDRESS:
      bDeviceState = ADDRESSED;
      return USB_SUCCESS;
    case USB_EVENT_CONFIG:
      current_conf = usb_lld_current_configuration ();
      if (current_conf == 0)
	{
	  if (value != 1)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (1);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    vcom_setup_endpoints_for_interface (i, 0);
	  bDeviceState = CONFIGURED;
	}
      else if (current_conf != value)
	{
	  if (value != 0)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (0);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    vcom_setup_endpoints_for_interface (i, 1);
	  bDeviceState = ADDRESSED;
	}
      /* Do nothing when current_conf == value */
      return USB_SUCCESS;

      return USB_SUCCESS;
    default:
      break;
    }

  return USB_UNSUPPORT;
}


int usb_cb_interface (uint8_t cmd, struct control_info *detail)
{
  const uint8_t zero = 0;
  uint16_t interface = detail->index;
  uint16_t alt = detail->value;

  if (interface >= NUM_INTERFACES)
    return USB_UNSUPPORT;

  switch (cmd)
    {
    case USB_SET_INTERFACE:
      if (alt != 0)
	return USB_UNSUPPORT;
      else
	{
	  vcom_setup_endpoints_for_interface (interface, 0);
	  return USB_SUCCESS;
	}

    case USB_GET_INTERFACE:
      return usb_lld_reply_request (&zero, 1, detail);

    default:
    case USB_QUERY_INTERFACE:
      return USB_SUCCESS;
    }
}

void
EP1_IN_Callback (void)
{
  chopstx_mutex_lock (&usb_mtx);
  chopstx_cond_signal (&cnd_usb);
  chopstx_mutex_unlock (&usb_mtx);
}

void
EP2_IN_Callback (void)
{
}

void
EP3_OUT_Callback (void)
{
  usb_lld_rx_enable (ENDP3);
}
