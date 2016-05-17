#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <string.h>
#include "usb_lld.h"
#include "tty.h"

#define BUFSIZE 128

struct tty {
  chopstx_mutex_t mtx;
  chopstx_cond_t cnd;
  chopstx_intr_t intr;
  uint8_t inputline[BUFSIZE];   /* Line editing is supported */
  uint8_t send_buf[BUFSIZE];	/* Sending ring buffer for echo back */
  uint32_t inputline_len    : 8;
  uint32_t send_head        : 8;
  uint32_t send_tail        : 8;
  uint32_t flag_connected   : 1;
  uint32_t flag_send_ready  : 1;
  uint32_t flag_input_avail : 1;
  uint32_t                  : 2;
  uint32_t device_state     : 3;     /* USB device status */
};

static struct tty tty;

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
  DEVICE_DESCRIPTOR,		/* bDescriptorType */
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

#define VCOM_FEATURE_BUS_POWERED	0x80

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_config_desc[67] = {
  9,
  CONFIG_DESCRIPTOR,		/* bDescriptorType: Configuration */
  /* Configuration Descriptor.*/
  67, 0x00,			/* wTotalLength.                    */
  0x02,				/* bNumInterfaces.                  */
  0x01,				/* bConfigurationValue.             */
  0,				/* iConfiguration.                  */
  VCOM_FEATURE_BUS_POWERED,	/* bmAttributes.                    */
  50,				/* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  9,
  INTERFACE_DESCRIPTOR,
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
  ENDPOINT_DESCRIPTOR,
  ENDP2|0x80,    /* bEndpointAddress.    */
  0x03,          /* bmAttributes (Interrupt).        */
  0x08, 0x00,	 /* wMaxPacketSize.                  */
  0xFF,		 /* bInterval.                       */
  /* Interface Descriptor.*/
  9,
  INTERFACE_DESCRIPTOR, /* bDescriptorType: */
  0x01,          /* bInterfaceNumber.                */
  0x00,          /* bAlternateSetting.               */
  0x02,          /* bNumEndpoints.                   */
  0x0A,          /* bInterfaceClass (Data Class Interface, CDC section 4.5). */
  0x00,          /* bInterfaceSubClass (CDC section 4.6). */
  0x00,          /* bInterfaceProtocol (CDC section 4.7). */
  0x00,		 /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,		/* bDescriptorType: Endpoint */
  ENDP3,    /* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00,				/* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,		/* bDescriptorType: Endpoint */
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
  STRING_DESCRIPTOR,
  0x09, 0x04			/* LangID = 0x0409: US-English */
};

static const uint8_t vcom_string1[] = {
  23*2+2,			/* bLength */
  STRING_DESCRIPTOR,		/* bDescriptorType */
  /* Manufacturer: "Flying Stone Technology" */
  'F', 0, 'l', 0, 'y', 0, 'i', 0, 'n', 0, 'g', 0, ' ', 0, 'S', 0,
  't', 0, 'o', 0, 'n', 0, 'e', 0, ' ', 0, 'T', 0, 'e', 0, 'c', 0,
  'h', 0, 'n', 0, 'o', 0, 'l', 0, 'o', 0, 'g', 0, 'y', 0, 
};

static const uint8_t vcom_string2[] = {
  14*2+2,			/* bLength */
  STRING_DESCRIPTOR,		/* bDescriptorType */
  /* Product name: "Chopstx Sample" */
  'C', 0, 'h', 0, 'o', 0, 'p', 0, 's', 0, 't', 0, 'x', 0, ' ', 0,
  'S', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0,
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[28] = {
  28,				    /* bLength */
  STRING_DESCRIPTOR,		    /* bDescriptorType */
  '0', 0,  '.', 0,  '0', 0, '0', 0, /* Version number */
};


#define NUM_INTERFACES 2


void
usb_cb_device_reset (void)
{
  usb_lld_reset (VCOM_FEATURE_BUS_POWERED);

  /* Initialize Endpoint 0 */
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0, ENDP0_RXADDR, ENDP0_TXADDR, 64);

  chopstx_mutex_lock (&tty.mtx);
  tty.inputline_len = 0;
  tty.send_head = tty.send_tail = 0;
  tty.flag_connected = 0;
  tty.flag_send_ready = 1;
  tty.flag_input_avail = 0;
  tty.device_state = ATTACHED;
  chopstx_mutex_unlock (&tty.mtx);
}


#define CDC_CTRL_DTR            0x0001

void
usb_cb_ctrl_write_finish (uint8_t req, uint8_t req_no, struct req_args *arg)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT)
      && USB_SETUP_SET (req) && req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
    {
      /* Open/close the connection.  */
      chopstx_mutex_lock (&tty.mtx);
      tty.flag_connected = ((arg->value & CDC_CTRL_DTR) != 0);
      chopstx_cond_signal (&tty.cnd);
      chopstx_mutex_unlock (&tty.mtx);
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
vcom_port_data_setup (uint8_t req, uint8_t req_no, struct req_args *arg)
{
  if (USB_SETUP_GET (req))
    {
      if (req_no == USB_CDC_REQ_GET_LINE_CODING)
	return usb_lld_reply_request (&line_coding, sizeof(line_coding), arg);
    }
  else  /* USB_SETUP_SET (req) */
    {
      if (req_no == USB_CDC_REQ_SET_LINE_CODING
	  && arg->len == sizeof (line_coding))
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
usb_cb_setup (uint8_t req, uint8_t req_no, struct req_args *arg)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT) && arg->index == 0)
    return vcom_port_data_setup (req, req_no, arg);

  return USB_UNSUPPORT;
}

int
usb_cb_get_descriptor (uint8_t rcp, uint8_t desc_type, uint8_t desc_index,
		       struct req_args *arg)
{
  if (rcp != DEVICE_RECIPIENT)
    return USB_UNSUPPORT;

  if (desc_type == DEVICE_DESCRIPTOR)
    return usb_lld_reply_request (vcom_device_desc, sizeof (vcom_device_desc),
				  arg);
  else if (desc_type == CONFIG_DESCRIPTOR)
    return usb_lld_reply_request (vcom_config_desc, sizeof (vcom_config_desc),
				  arg);
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

      return usb_lld_reply_request (str, size, arg);
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
	  /* Start with no data receiving */
	  usb_lld_stall_rx (ENDP3);
	}
      else
	{
	  usb_lld_stall_tx (ENDP1);
	  usb_lld_stall_rx (ENDP3);
	}
    }
}

int
usb_cb_handle_event (uint8_t event_type, uint16_t value)
{
  int i;
  uint8_t current_conf;

  switch (event_type)
    {
    case USB_EVENT_ADDRESS:
      chopstx_mutex_lock (&tty.mtx);
      tty.device_state = ADDRESSED;
      chopstx_mutex_unlock (&tty.mtx);
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
	  chopstx_mutex_lock (&tty.mtx);
	  tty.device_state = CONFIGURED;
	  chopstx_mutex_unlock (&tty.mtx);
	}
      else if (current_conf != value)
	{
	  if (value != 0)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (0);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    vcom_setup_endpoints_for_interface (i, 1);
	  chopstx_mutex_lock (&tty.mtx);
	  tty.device_state = ADDRESSED;
	  chopstx_mutex_unlock (&tty.mtx);
	}
      /* Do nothing when current_conf == value */
      return USB_SUCCESS;

      return USB_SUCCESS;
    default:
      break;
    }

  return USB_UNSUPPORT;
}


int
usb_cb_interface (uint8_t cmd, struct req_args *arg)
{
  const uint8_t zero = 0;
  uint16_t interface = arg->index;
  uint16_t alt = arg->value;

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
      return usb_lld_reply_request (&zero, 1, arg);

    default:
    case USB_QUERY_INTERFACE:
      return USB_SUCCESS;
    }
}


/*
 * Put a character into the ring buffer to be send back.
 */
static void
put_char_to_ringbuffer (int c)
{
  uint32_t next = (tty.send_tail + 1) % BUFSIZE;

  if (tty.send_head == next)
    /* full */
    /* All that we can do is ignore this char. */
    return;
  
  tty.send_buf[tty.send_tail] = c;
  tty.send_tail = next;
}

/*
 * Get characters from ring buffer into S.
 */
static int
get_chars_from_ringbuffer (uint8_t *s, int len)
{
  int i = 0;

  if (tty.send_head == tty.send_tail)
    /* Empty */
    return i;

  do
    {
      s[i++] = tty.send_buf[tty.send_head];
      tty.send_head = (tty.send_head + 1) % BUFSIZE;
    }
  while (tty.send_head != tty.send_tail && i < len);

  return i;
}


static void
tty_echo_char (int c)
{
  put_char_to_ringbuffer (c);
}


void
usb_cb_tx_done (uint8_t ep_num)
{
  if (ep_num == ENDP1)
    {
      chopstx_mutex_lock (&tty.mtx);
      if (tty.flag_send_ready == 0)
	{
	  tty.flag_send_ready = 1;
	  chopstx_cond_signal (&tty.cnd);
	}
      chopstx_mutex_unlock (&tty.mtx);
    }
  else if (ep_num == ENDP2)
    {
      /* Nothing */
    }
}


static int
tty_input_char (int c)
{
  unsigned int i;
  int r = 0;

  /* Process DEL, C-U, C-R, and RET as editing command. */
  chopstx_mutex_lock (&tty.mtx);
  switch (c)
    {
    case 0x0d: /* Control-M */
      tty_echo_char (0x0d);
      tty_echo_char (0x0a);
      tty.flag_input_avail = 1;
      r = 1;
      chopstx_cond_signal (&tty.cnd);
      break;
    case 0x12: /* Control-R */
      tty_echo_char ('^');
      tty_echo_char ('R');
      tty_echo_char (0x0d);
      tty_echo_char (0x0a);
      for (i = 0; i < tty.inputline_len; i++)
	tty_echo_char (tty.inputline[i]);
      break;
    case 0x15: /* Control-U */
      for (i = 0; i < tty.inputline_len; i++)
	{
	  tty_echo_char (0x08);
	  tty_echo_char (0x20);
	  tty_echo_char (0x08);
	}
      tty.inputline_len = 0;
      break;
    case 0x7f: /* DEL    */
      if (tty.inputline_len > 0)
	{
	  tty_echo_char (0x08);
	  tty_echo_char (0x20);
	  tty_echo_char (0x08);
	  tty.inputline_len--;
	}
      break;
    default:
      if (tty.inputline_len < sizeof (tty.inputline))
	{
	  tty_echo_char (c);
	  tty.inputline[tty.inputline_len++] = c;
	}
      else
	/* Beep */
	tty_echo_char (0x0a);
      break;
    }
  chopstx_mutex_unlock (&tty.mtx);
  return r;
}

void
usb_cb_rx_ready (uint8_t ep_num)
{
  uint8_t recv_buf[64];

  if (ep_num == ENDP3)
    {
      int i, r;

      r = usb_lld_rx_data_len (ENDP3);
      usb_lld_rxcpy (recv_buf, ep_num, 0, r);
      for (i = 0; i < r; i++)
	if (tty_input_char (recv_buf[i]))
	  break;

      chopstx_mutex_lock (&tty.mtx);
      if (tty.flag_input_avail == 0)
	usb_lld_rx_enable (ENDP3);
      chopstx_mutex_unlock (&tty.mtx);
    }
}

static void *tty_main (void *arg);

#define INTR_REQ_USB 20
#define PRIO_TTY      4

extern uint8_t __process3_stack_base__, __process3_stack_size__;
const uint32_t __stackaddr_tty = (uint32_t)&__process3_stack_base__;
const size_t __stacksize_tty = (size_t)&__process3_stack_size__;

struct tty *
tty_open (void)
{
  chopstx_mutex_init (&tty.mtx);
  tty.inputline_len = 0;
  tty.send_head = tty.send_tail = 0;
  tty.flag_connected = 0;
  tty.flag_send_ready = 1;
  tty.flag_input_avail = 0;
  tty.device_state = UNCONNECTED;

  chopstx_create (PRIO_TTY, __stackaddr_tty, __stacksize_tty,
		  tty_main, NULL);
  return &tty;
}


static void *
tty_main (void *arg)
{
  (void)arg;

#if defined(OLDER_SYS_H)
  /*
   * Historically (before sys < 3.0), NVIC priority setting for USB
   * interrupt was done in usb_lld_sys_init.  Thus this code.
   *
   * When USB interrupt occurs between usb_lld_init (which assumes
   * ISR) and chopstx_claim_irq (which clears pending interrupt),
   * invocation of usb_interrupt_handler won't occur.
   *
   * Calling usb_interrupt_handler is no harm even if there were no
   * interrupts, thus, we call it unconditionally here, just in case
   * if there is a request.
   *
   * We can't call usb_lld_init after chopstx_claim_irq, as
   * usb_lld_init does its own setting for NVIC.  Calling
   * chopstx_claim_irq after usb_lld_init overrides that.
   *
   */
  usb_lld_init (VCOM_FEATURE_BUS_POWERED);
  chopstx_claim_irq (&tty.intr, INTR_REQ_USB);
  usb_interrupt_handler ();
#else
  chopstx_claim_irq (&tty.intr, INTR_REQ_USB);
  usb_lld_init (VCOM_FEATURE_BUS_POWERED);
#endif

  while (1)
    {
      chopstx_poll (NULL, 1, &tty.intr);
      if (tty.intr.ready)
	usb_interrupt_handler ();

      chopstx_mutex_lock (&tty.mtx);
      if (tty.device_state == CONFIGURED && tty.flag_connected
	  && tty.flag_send_ready)
	{
	  uint8_t line[32];
	  int len = get_chars_from_ringbuffer (line, sizeof (len));

	  if (len)
	    {
	      usb_lld_txcpy (line, ENDP1, 0, len);
	      usb_lld_tx_enable (ENDP1, len);
	      tty.flag_send_ready = 0;
	    }
	}
      chopstx_mutex_unlock (&tty.mtx);
    }

  return NULL;
}


void
tty_wait_configured (struct tty *t)
{
  chopstx_mutex_lock (&t->mtx);
  while (t->device_state != CONFIGURED)
    chopstx_cond_wait (&t->cnd, &t->mtx);
  chopstx_mutex_unlock (&t->mtx);
}


void
tty_wait_connection (struct tty *t)
{
  chopstx_mutex_lock (&t->mtx);
  while (t->flag_connected == 0)
    chopstx_cond_wait (&t->cnd, &t->mtx);
  t->flag_send_ready = 1;
  t->flag_input_avail = 0;
  t->send_head = t->send_tail = 0;
  t->inputline_len = 0;
  usb_lld_rx_enable (ENDP3);	/* Accept input for line */
  chopstx_mutex_unlock (&t->mtx);
}

static int
check_tx (struct tty *t)
{
  if (t->flag_send_ready)
    /* TX done */
    return 1;
  if (t->flag_connected == 0)
    /* Disconnected */
    return -1;
  return 0;
}

int
tty_send (struct tty *t, uint8_t *buf, int len)
{
  int r;
  uint8_t *p;
  int count;

  p = buf;
  count = len >= 64 ? 64 : len;

  while (1)
    {
      chopstx_mutex_lock (&t->mtx);
      while ((r = check_tx (t)) == 0)
	chopstx_cond_wait (&t->cnd, &t->mtx);
      if (r > 0)
	{
	  usb_lld_txcpy (p, ENDP1, 0, count);
	  usb_lld_tx_enable (ENDP1, count);
	  t->flag_send_ready = 0;
	}
      chopstx_mutex_unlock (&t->mtx);

      len -= count;
      p += count;
      if (len == 0 && count != 64)
	/*
	 * The size of the last packet should be != 0
	 * If 64, send ZLP (zelo length packet)
	 */
	break;
      count = len >= 64 ? 64 : len;
    }

  return r;
}


static int
check_rx (void *arg)
{
  struct tty *t = arg;

  if (t->flag_input_avail)
    /* RX */
    return 1;
  if (t->flag_connected == 0)
    /* Disconnected */
    return 1;
  return 0;
}

int
tty_recv (struct tty *t, uint8_t *buf, uint32_t *timeout)
{
  int r;
  chopstx_poll_cond_t poll_desc;

  poll_desc.type = CHOPSTX_POLL_COND;
  poll_desc.ready = 0;
  poll_desc.cond = &t->cnd;
  poll_desc.mutex = &t->mtx;
  poll_desc.check = check_rx;
  poll_desc.arg = t;

  while (1)
    {
      chopstx_poll (timeout, 1, &poll_desc);
      chopstx_mutex_lock (&t->mtx);
      r = check_rx (t);
      chopstx_mutex_unlock (&t->mtx);
      if (r || (timeout != NULL && *timeout == 0))
	break;
    }

  chopstx_mutex_lock (&t->mtx);
  if (t->flag_connected == 0)
    r = -1;
  else if (t->flag_input_avail)
    {
      r = t->inputline_len;
      memcpy (buf, t->inputline, r);
      t->flag_input_avail = 0;
      usb_lld_rx_enable (ENDP3);
      t->inputline_len = 0;
    }
  else
    r = 0;
  chopstx_mutex_unlock (&t->mtx);

  return r;
}
