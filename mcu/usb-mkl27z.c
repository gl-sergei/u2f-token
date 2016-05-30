/*
 * usb-mkl27z.c - USB driver for MKL27Z
 *
 * Copyright (C) 2016  Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
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
 * receipents of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "usb_lld.h"

struct endpoint_ctl {
  uint32_t rx_odd: 1;
  uint32_t tx_odd: 1;
};
static struct endpoint_ctl ep[16];

struct USB_CONF {
  const uint8_t PERID;       /* Peripheral ID register              */
  uint8_t rsvd0[3];          /*                                     */
  const uint8_t IDCOMP;      /* Peripheral ID Complement register   */
  uint8_t rsvd1[3];          /*                                     */
  const uint8_t REV;         /* Peripheral Revision register        */
  uint8_t rsvd2[3];          /*                                     */
  volatile uint8_t ADDINFO;  /* Peripheral Additional Info register */
};
static struct USB_CONF *const USB_CONF = (struct USB_CONF *const) 0x40072000;

struct USB_CTRL0 {
  volatile uint8_t OTGCTL;   /* OTG Control register                */
};
static struct USB_CTRL0 *const USB_CTRL0 = (struct USB_CTRL0 *const)0x4007201c;

struct USB_CTRL1 {
  volatile uint8_t ISTAT;    /* Interrupt Status register           */
  uint8_t rsvd5[3];          /*                                     */
  volatile uint8_t INTEN;    /* Interrupt Enable register           */
  uint8_t rsvd6[3];          /*                                     */
  volatile uint8_t ERRSTAT;  /* Error Interrupt Status register     */
  uint8_t rsvd7[3];          /*                                     */
  volatile uint8_t ERREN;    /* Error Interrupt Enable register     */
  uint8_t rsvd8[3];          /*                                     */
  volatile uint8_t STAT;     /* Status register                     */
  uint8_t rsvd9[3];          /*                                     */
  volatile uint8_t CTL;      /* Control register                    */
  uint8_t rsvd10[3];         /*                                     */
  volatile uint8_t ADDR;     /* Address register                    */
  uint8_t rsvd11[3];         /*                                     */
  volatile uint8_t BDTPAGE1; /* BDT Page register 1                 */
  uint8_t rsvd12[3];         /*                                     */
  volatile uint8_t FRMNUML;  /* Frame Number register Low           */
  uint8_t rsvd13[3];         /*                                     */
  volatile uint8_t FRMNUMH;  /* Frame Number register High          */
  uint8_t rsvd14[11];        /*                                     */
  volatile uint8_t BDTPAGE2; /* BDT Page Register 2                 */
  uint8_t rsvd15[3];         /*                                     */
  volatile uint8_t BDTPAGE3; /* BDT Page Register 3                 */
};
static struct USB_CTRL1 *const USB_CTRL1 = (struct USB_CTRL1 *const)0x40072080;

/* Interrupt source bits */
#define USB_IS_STALL  (1 << 7)
#define USB_IS_RESUME (1 << 5)
#define USB_IS_SLEEP  (1 << 4)
#define USB_IS_TOKDNE (1 << 3)
#define USB_IS_SOFTOK (1 << 2)
#define USB_IS_ERROR  (1 << 1)
#define USB_IS_USBRST (1 << 0)


struct USB_ENDPT {
  volatile uint8_t EP;  /* Endpoint Control register */
  uint8_t rsvd17[3];
};
static struct USB_ENDPT *const USB_ENDPT = (struct USB_ENDPT *const)0x400720c0;

struct USB_CTRL2 {
  volatile uint8_t USBCTRL;  /* USB Control register                */
  uint8_t rsvd33[3];         /*                                     */
  volatile uint8_t OBSERVE;  /* USB OTG Observe register            */
  uint8_t rsvd34[3];         /*                                     */
  volatile uint8_t CONTROL;  /* USB OTG Control register            */
  uint8_t rsvd35[3];         /*                                     */
  volatile uint8_t USBTRC0;  /* USB Transceiver Control register 0  */
  uint8_t rsvd36[7];         /*                                     */
  volatile uint8_t USBFRMADJUST;           /* Frame Adjut Register  */
};
static struct USB_CTRL2 *const USB_CTRL2 = (struct USB_CTRL2 *const)0x40072100;

/* Buffer Descriptor */
struct BD {
  volatile uint32_t ctrl;
  volatile void *buf; 
};
/*
  uint32_t rsvd0 : 2;
  volatile uint32_t STALL: 1;
  volatile uint32_t DTS: 1;

  volatile uint32_t NINC: 1;
  volatile uint32_t KEEP: 1;
  volatile uint32_t DATA01: 1;
  volatile uint32_t OWN: 1;

  uint32_t rsvd1: 8;
  volatile uint32_t BC: 10;
  uint32_t rsvd2: 6;
*/
#define TOK_PID(ctrl)   ((ctrl >> 2) & 0x0f)

extern uint8_t __usb_bdt__;

static struct BD *const BD_table = (struct BD *const)&__usb_bdt__;

static uint8_t setup[8];
/* bmRequestType, bRequest */
/* Value: 2-byte  */
/* Index: 2-byte  */
/* Length: 2-byte */

static void
kl27z_usb_init (void)
{
  int i;

  memset (ep, 0, sizeof (ep));
  memset (BD_table, 0, 16 * 2 * 2 * sizeof (struct BD));

  /* D+ pull up */
  USB_CTRL0->OTGCTL = 0x80;

  USB_CTRL1->ERREN = 0xff;

  USB_CTRL1->BDTPAGE1 = ((uint32_t)BD_table) >> 8;
  USB_CTRL1->BDTPAGE2 = ((uint32_t)BD_table) >> 16;
  USB_CTRL1->BDTPAGE3 = ((uint32_t)BD_table) >> 24;

  /* Not suspended, Pull-down disabled.  */
  USB_CTRL2->USBCTRL = 0x00;
  /* DP Pullup in non-OTG device mode.  */
  USB_CTRL2->CONTROL = 0x10;

  /* Disable all endpoints.  */
  for (i = 0; i < 16; i++)
    USB_ENDPT[i].EP = 0;

  /* 
   * Enable USB FS communication module, clearing all ODD-bits
   * for BDT.
   */
  USB_CTRL1->CTL = 0x03;    

  /* ??? How we can ask re-enumeration?  Is only hard RESET enough? */
}

static void
kl27z_set_daddr (uint8_t daddr)
{
  USB_CTRL1->ADDR = daddr;
}

static void
kl27z_prepare_ep0_setup (void)
{
  /* Endpoint 0, TX=0. */
  BD_table[ep[0].rx_odd].ctrl = 0x00080088; /* Len=8, OWN=1, DATA01=0, DTS=1 */
  BD_table[ep[0].rx_odd].buf = setup;

  BD_table[!ep[0].rx_odd].ctrl = 0x0000; /* OWN=0 */
  BD_table[!ep[0].rx_odd].buf = NULL;
}

static void
kl27z_prepare_ep0_in (const void *buf, uint8_t len, int data01)
{
  /* Endpoint 0, TX=1 *//* OWN=1, DTS=1 */
  BD_table[2+ep[0].tx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6); 
  BD_table[2+ep[0].tx_odd].buf = (void *)buf;
}

static void
kl27z_prepare_ep0_out (void *buf, uint8_t len, int data01)
{
  /* Endpoint 0, TX=0 *//* OWN=1, DTS=1 */
  BD_table[ep[0].rx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6);
  BD_table[ep[0].rx_odd].buf = buf;
}

static int
kl27z_ep_is_disabled (uint8_t n)
{
  return (USB_ENDPT[n].EP == 0);
}

static int
kl27z_ep_is_stall (uint8_t n)
{
  return (USB_ENDPT[n].EP & 0x02) >> 1;
}

static void
kl27z_ep_stall (uint8_t n)
{
  USB_ENDPT[n].EP |= 0x02;
}

static void
kl27z_ep_clear_stall (uint8_t n)
{
  USB_ENDPT[n].EP &= ~0x02;
}

static void
kl27z_ep_clear_dtog (int rx, uint8_t n)
{
  uint32_t config;

  if (!kl27z_ep_is_stall (n))
    /* Just in case, when the endpoint is active */
    kl27z_ep_stall (n);

  if (rx)
    {
      config = BD_table[4*n+ep[n].rx_odd].ctrl;

      BD_table[4*n+!ep[n].rx_odd].ctrl &= ~(1 << 6);
      if ((config & 0x0080)) /* OWN already? */
	{
	  /*
	   * How to update BDT entry which is owned by USBFS seems to
	   * be not clearly documented.  It would be just OK to update
	   * it as long as the endpoint is stalled (BDT entry is
	   * actually not in use).  We write 0 at first and then write
	   * value with OWN, to avoid possible failure.
	   */
	  BD_table[4*n+ep[n].rx_odd].ctrl = 0;
	  BD_table[4*n+ep[n].rx_odd].ctrl = (config & ~(1 << 6));
	}
    }
  else
    {
      config = BD_table[4*n+2+ep[n].tx_odd].ctrl;

      BD_table[4*n+2+!ep[n].tx_odd].ctrl &= ~(1 << 6);
      if ((config & 0x0080)) /* OWN already? */
	{
	  BD_table[4*n+2+ep[n].tx_odd].ctrl = 0;
	  BD_table[4*n+2+ep[n].tx_odd].ctrl = (config & ~(1 << 6));
	}
    }

  kl27z_ep_clear_stall (n);
}

#define USB_MAX_PACKET_SIZE 64	/* For FS device */

enum STANDARD_REQUESTS {
  GET_STATUS = 0,
  CLEAR_FEATURE,
  RESERVED1,
  SET_FEATURE,
  RESERVED2,
  SET_ADDRESS,
  GET_DESCRIPTOR,
  SET_DESCRIPTOR,
  GET_CONFIGURATION,
  SET_CONFIGURATION,
  GET_INTERFACE,
  SET_INTERFACE,
  SYNCH_FRAME,
  TOTAL_REQUEST  /* Total number of Standard request */
};


enum FEATURE_SELECTOR {
  ENDPOINT_STALL,
  DEVICE_REMOTE_WAKEUP
};


struct data_ctl {
  uint8_t *addr;
  uint16_t len;
  uint8_t require_zlp;
};

/* The state machine states of a control pipe */
enum {
  WAIT_SETUP,
  IN_DATA,
  OUT_DATA,
  LAST_IN_DATA,
  WAIT_STATUS_IN,
  WAIT_STATUS_OUT,
  STALLED,
  PAUSE
};

struct device_ctl {
  /* control pipe state */
  uint8_t state;		

  uint32_t tkdone;
  uint32_t reset;
  uint32_t error;
  uint32_t stall;

  uint32_t send;
  uint32_t recv;

  /* Device specific settings */
  uint8_t configuration;
  uint8_t feature;
};

static struct device_ctl device_ctl;
static struct data_ctl data_ctl;

static struct device_ctl *const dev_p = &device_ctl;
static struct data_ctl *const data_p = &data_ctl;

static void handle_transaction (uint8_t stat);

void
usb_lld_stall (int n)
{
  kl27z_ep_stall (n);
}


void
usb_lld_init (uint8_t feature)
{
  dev_p->state = WAIT_SETUP;
  dev_p->tkdone = 0;
  dev_p->reset = 0;
  dev_p->error = 0;
  dev_p->stall = 0;

  usb_lld_set_configuration (0);
  dev_p->feature = feature;

  kl27z_set_daddr (0);
  kl27z_usb_init ();

  /* Enable the endpoint 0.  */
  USB_ENDPT[0].EP = 0x0d;

  /* Clear Interrupt Status Register, and enable interrupt for USB */
  USB_CTRL1->ISTAT = 0xff;		     /* All clear */

  USB_CTRL1->INTEN = USB_IS_STALL | USB_IS_TOKDNE
                   | USB_IS_ERROR | USB_IS_USBRST;
}

void
usb_interrupt_handler (void)
{
  uint8_t istat_value = USB_CTRL1->ISTAT;
  uint8_t stat = USB_CTRL1->STAT;

  if ((istat_value & USB_IS_USBRST))
    {
      USB_CTRL1->ISTAT = USB_IS_USBRST;
      usb_cb_device_reset ();
      dev_p->reset++;
    }
  else if ((istat_value & USB_IS_TOKDNE))
    {
      handle_transaction (stat);
      dev_p->tkdone++;
    }
  else if ((istat_value & USB_IS_ERROR))
    { /* Clear Errors.  */
      USB_CTRL1->ERRSTAT = USB_CTRL1->ERRSTAT;
      USB_CTRL1->ISTAT = USB_IS_ERROR;
      /*reset???*/
      dev_p->error++;
    }
  else if ((istat_value & USB_IS_STALL))
    {
      /* Does STAT have ENDPOINT info in this case?: No, it doesn't.  */

      if (kl27z_ep_is_stall (0))
	{ /* It's endpoint 0, recover from erorr.  */
	  dev_p->state = WAIT_SETUP;
	  kl27z_ep_clear_stall (0);
	  kl27z_prepare_ep0_setup ();
	}

      USB_CTRL1->ISTAT = USB_IS_STALL;
      dev_p->stall++;
    }
}

#define DATA0 0
#define DATA1 1

static void
handle_datastage_out (uint8_t stat)
{
  int odd = (stat >> 2)&1;
  int data01 = !((BD_table[odd].ctrl >> 6)&1);
  uint32_t len = (BD_table[odd].ctrl >> 16)&0x3ff;

  data_p->len -= len;
  data_p->addr += len;

  len = data_p->len;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;

  if (data_p->len == 0)
    {
      /* No more data to receive, proceed to send acknowledge for IN.  */
      dev_p->state = WAIT_STATUS_IN;
      kl27z_prepare_ep0_in (setup, 0, DATA1);
    }
  else
    {
      dev_p->state = OUT_DATA;
      kl27z_prepare_ep0_out (data_p->addr, len, data01);
    }
}

static void
handle_datastage_in (uint8_t stat)
{
  int odd = (stat >> 2)&1;
  int data01 = !((BD_table[2+odd].ctrl >> 6)&1);
  uint32_t len = USB_MAX_PACKET_SIZE;

  if ((data_p->len == 0) && (dev_p->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  kl27z_prepare_ep0_in (setup, 0, data01);
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.  */
	  dev_p->state = WAIT_STATUS_OUT;
	  kl27z_prepare_ep0_out (setup, 8, DATA1);
	}

      return;
    }

  dev_p->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  kl27z_prepare_ep0_in (data_p->addr, len, data01);
  data_p->len -= len;
  data_p->addr += len;
}

typedef int (*HANDLER) (uint8_t req, struct req_args *arg);

static int
std_none (uint8_t req, struct req_args *arg)
{
  (void)req; (void)arg;
  return USB_UNSUPPORT;
}

static int
std_get_status (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;
  uint16_t status_info = 0;

  if (arg->value != 0 || arg->len != 2 || (arg->index >> 8) != 0
      || USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->index == 0)
	{
	  /* Get Device Status */
	  uint8_t feature = dev_p->feature;

	  /* Remote Wakeup enabled */
	  if ((feature & (1 << 5)))
	    status_info |= 2;
	  else
	    status_info &= ~2;

	  /* Bus-powered */
	  if ((feature & (1 << 6)))
	    status_info |= 1;
	  else /* Self-powered */
	    status_info &= ~1;

	  return usb_lld_reply_request (&status_info, 2, arg);
	}
    }
  else if (rcp == INTERFACE_RECIPIENT)
    {
      int r;

      if (dev_p->configuration == 0)
	return USB_UNSUPPORT;

      r = usb_cb_interface (USB_QUERY_INTERFACE, arg);
      if (r != USB_SUCCESS)
	return USB_UNSUPPORT;

      return usb_lld_reply_request (&status_info, 2, arg);
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if ((arg->index & 0x70) || n == ENDP0)
	return USB_UNSUPPORT;

      if (kl27z_ep_is_disabled (n))
	return USB_UNSUPPORT;

      status_info = kl27z_ep_is_stall (n);
      return usb_lld_reply_request (&status_info, 2, arg);
    }

  return USB_UNSUPPORT;
}

static int
std_clear_feature (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return USB_UNSUPPORT;

      if (arg->value == DEVICE_REMOTE_WAKEUP)
	{
	  dev_p->feature &= ~(1 << 5);
	  return USB_SUCCESS;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev_p->configuration == 0)
	return USB_UNSUPPORT;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != ENDPOINT_STALL || n == ENDP0)
	return USB_UNSUPPORT;

      if (kl27z_ep_is_disabled (n))
	return USB_UNSUPPORT;

      kl27z_ep_clear_dtog ((arg->index & 0x80) == 0, n);

      // event??
      return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

static int
std_set_feature (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return USB_UNSUPPORT;

      if (arg->value == DEVICE_REMOTE_WAKEUP)
	{
	  dev_p->feature |= 1 << 5;
	  // event??
	  return USB_SUCCESS;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev_p->configuration == 0)
	return USB_UNSUPPORT;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != 0 || n == ENDP0)
	return USB_UNSUPPORT;

      if (kl27z_ep_is_disabled (n))
	return USB_UNSUPPORT;

      kl27z_ep_stall (n);

      // event??
      return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

static int
std_set_address (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT && arg->len == 0 && arg->value <= 127
      && arg->index == 0 && dev_p->configuration == 0)
    return USB_SUCCESS;

  return USB_UNSUPPORT;
}

static int
std_get_descriptor (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  return usb_cb_get_descriptor (rcp, (arg->value >> 8),
				(arg->value & 0xff), arg);
}

static int
std_get_configuration (uint8_t req,  struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  (void)arg;
  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT)
    return usb_lld_reply_request (&dev_p->configuration, 1, arg);

  return USB_UNSUPPORT;
}

static int
std_set_configuration (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT && arg->index == 0 && arg->len == 0)
    return usb_cb_handle_event (USB_EVENT_CONFIG, arg->value);

  return USB_UNSUPPORT;
}

static int
std_get_interface (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  if (rcp == INTERFACE_RECIPIENT)
    {
      if (arg->value != 0 || (arg->index >> 8) != 0 || arg->len != 1)
	return USB_UNSUPPORT;

      if (dev_p->configuration == 0)
	return USB_UNSUPPORT;

      return usb_cb_interface (USB_GET_INTERFACE, arg);
    }

  return USB_UNSUPPORT;
}

static int
std_set_interface (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev_p->configuration == 0)
    return USB_UNSUPPORT;

  return usb_cb_interface (USB_SET_INTERFACE, arg);
}


static void
handle_setup0 (void)
{
  struct req_args *arg = (struct req_args *)&setup[2];
  int r = USB_UNSUPPORT;
  HANDLER handler;

  data_p->addr = NULL;
  data_p->len = 0;
  data_p->require_zlp = 0;

  if ((setup[0] & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      if (setup[1] < TOTAL_REQUEST)
	{
	  switch (setup[1])
	    {
	    case 0: handler = std_get_status;  break;
	    case 1: handler = std_clear_feature;  break;
	    case 3: handler = std_set_feature;  break;
	    case 5: handler = std_set_address;  break;
	    case 6: handler = std_get_descriptor;  break;
	    case 8: handler = std_get_configuration;  break;
	    case 9: handler = std_set_configuration;  break;
	    case 10: handler = std_get_interface;  break;
	    case 11: handler = std_set_interface;  break;
	    default: handler = std_none;  break;
	    }

	  r = (*handler) (setup[0], arg);
	}
    }
  else
    r = usb_cb_setup (setup[0], setup[1], arg);

  if (r != USB_SUCCESS)
    dev_p->state = STALLED;
  else if (USB_SETUP_SET (setup[0]))
    {
      if (arg->len == 0)
	{
	  /* Zero length packet for ACK.  */
	  kl27z_prepare_ep0_in (setup, 0, DATA1);
	  dev_p->state = WAIT_STATUS_IN;
	}
    }
}

static void
handle_in0 (uint8_t stat)
{
  if (dev_p->state == IN_DATA || dev_p->state == LAST_IN_DATA)
    handle_datastage_in (stat);
  else if (dev_p->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
      uint16_t value = (setup[3]<<8) | setup[2];

      if ((setup[1] == SET_ADDRESS) &&
	  ((setup[0] & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	{
	  kl27z_set_daddr (value);
	  usb_cb_handle_event (USB_EVENT_ADDRESS, value);
	  ep[0].rx_odd = 0;
	}
      else
	usb_cb_ctrl_write_finish  (setup[0], setup[1],
				   (struct req_args *)&setup[2]);

      dev_p->state = WAIT_SETUP;
      kl27z_prepare_ep0_setup ();
    }
  else
    dev_p->state = STALLED;
}

static void
handle_out0 (uint8_t stat)
{
  if (dev_p->state == IN_DATA || dev_p->state == LAST_IN_DATA)
    /* Host aborts the control READ transfer before finish. */
    dev_p->state = STALLED;
  else if (dev_p->state == OUT_DATA)
    /* It's normal control WRITE transfer.  */
    handle_datastage_out (stat);
  else if (dev_p->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
      dev_p->state = WAIT_SETUP;
      kl27z_prepare_ep0_setup ();
    }
  else
    dev_p->state = STALLED;
}

#define USB_TOKEN_ACK   0x02
#define USB_TOKEN_IN    0x09
#define USB_TOKEN_SETUP 0x0d

static void
handle_transaction (uint8_t stat)
{
  int odd = (stat >> 2)&1;
  uint8_t ep_num = (stat >> 4);

  if (ep_num == 0)
    {
      if ((stat & 0x08) == 0)
	{
	  ep[0].rx_odd ^= 1;
	  if (TOK_PID (BD_table[odd].ctrl) == USB_TOKEN_SETUP)
	    {
	      handle_setup0 ();
	      USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	      USB_CTRL1->CTL = 0x01; /* Clear TXSUSPENDTOKENBUSY.  */
	    }
	  else
	    {
	      USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	      handle_out0 (stat);
	    }
	}
      else
	{
	  ep[0].tx_odd ^= 1;
	  USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	  handle_in0 (stat);
	}

      if (dev_p->state == STALLED)
	kl27z_ep_stall (0);
    }
  else
    {
      if ((stat & 0x08) == 0)
	{
	  dev_p->recv++;
	  ep[ep_num].rx_odd ^= 1;
	  usb_cb_rx_ready (ep_num);
	}
      else
	{
	  /*
	   * IN transaction is usually in a sequence like:
	   *
	   *           -----time------>
	   *   host:   IN          ACK
	   *   device:     DATA0/1 
	   *
	   * It is not described in the specification (it's
	   * ambiguous), but it is actually possible for some host
	   * implementation to send back a NAK on erroneous case like
	   * a device sent oversized data.
	   *
	   *           -----time------>
	   *   host:   IN          NAK
	   *   device:     DATA0/1 
	   *
	   * We do our best to distinguish successful tx and tx with
	   * failure.
	   *
	   */
	  uint32_t dmaerr = (USB_CTRL1->ERRSTAT & (1 << 5));
	  int success = (dmaerr == 0);
	  uint32_t len = (BD_table[4*ep_num+2+odd].ctrl >> 16)&0x3ff;

	  if (!success)
	    USB_CTRL1->ERRSTAT = dmaerr; /* Clear error.  */

	  dev_p->send++;
	  ep[ep_num].tx_odd ^= 1;
	  usb_cb_tx_done (ep_num, len, success);
	}

      USB_CTRL1->ISTAT = USB_IS_TOKDNE;
    }
}

void
usb_lld_reset (uint8_t feature)
{
  dev_p->feature = feature;
  usb_lld_set_configuration (0);

  /* Reset USB */
  USB_CTRL2->USBTRC0 = 0xc0;

  USB_CTRL1->CTL = 0x00;    /* Disable USB FS communication module */

  dev_p->state = WAIT_SETUP;
  dev_p->tkdone = 0;
  dev_p->error = 0;
  dev_p->stall = 0;

  kl27z_set_daddr (0);
  kl27z_usb_init ();

  /* Clear Interrupt Status Register, and enable interrupt for USB */
  USB_CTRL1->ISTAT = 0xff;		     /* All clear */

  USB_CTRL1->INTEN = USB_IS_STALL | USB_IS_TOKDNE
                   | USB_IS_ERROR | USB_IS_USBRST;
}

void
usb_lld_setup_endp (int n, int rx_en, int tx_en)
{
  if (n == 0)
    {
      /* Enable the endpoint 0.  */
      USB_ENDPT[0].EP = 0x0d;
      kl27z_prepare_ep0_setup ();
    }
  else
    {
      /* Enable the endpoint.  */
      USB_ENDPT[n].EP = (rx_en << 3)|(tx_en << 2)|0x11;

      /* Configure BDT entry so that it starts with DATA0.  */

      /* RX */
      BD_table[4*n+ep[n].rx_odd].ctrl = 0x0000;
      BD_table[4*n+ep[n].rx_odd].buf = NULL;
      BD_table[4*n+!ep[n].rx_odd].ctrl = 0x0040;
      BD_table[4*n+!ep[n].rx_odd].buf = NULL;

      /* TX */
      BD_table[4*n+2+ep[n].tx_odd].ctrl = 0x0000;
      BD_table[4*n+2+ep[n].tx_odd].buf = NULL;
      BD_table[4*n+2+!ep[n].tx_odd].ctrl = 0x0040;
      BD_table[4*n+2+!ep[n].tx_odd].buf = NULL;
    }
}


void
usb_lld_set_configuration (uint8_t config)
{
  dev_p->configuration = config;
}

uint8_t
usb_lld_current_configuration (void)
{
  return dev_p->configuration;
}

void
usb_lld_set_data_to_recv (void *p, size_t len)
{
  data_p->addr = (uint8_t *)p;
  data_p->len = len;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;

  kl27z_prepare_ep0_out (p, len, DATA1);
  dev_p->state = OUT_DATA;
}

/*
 * BUF: Pointer to data memory.  Data memory should not be allocated
 *      on stack when BUFLEN > USB_MAX_PACKET_SIZE.
 *
 * BUFLEN: size of the data.
 */
int
usb_lld_reply_request (const void *buf, size_t buflen, struct req_args *a)
{
  uint32_t len_asked = a->len;
  uint32_t len;

  data_p->addr = (void *)buf;
  data_p->len = buflen;

  /* Restrict the data length to be the one host asks for */
  if (data_p->len > len_asked)
    data_p->len = len_asked;

  data_p->require_zlp = (data_p->len != 0
			 && (data_p->len % USB_MAX_PACKET_SIZE) == 0);

  if (data_p->len < USB_MAX_PACKET_SIZE)
    {
      len = data_p->len;
      dev_p->state = LAST_IN_DATA;
    }
  else
    {
      len = USB_MAX_PACKET_SIZE;
      dev_p->state = IN_DATA;
    }

  if (len)
    kl27z_prepare_ep0_in (data_p->addr, len, DATA1);

  data_p->len -= len;
  data_p->addr += len;

  return USB_SUCCESS;
}

void
usb_lld_rx_enable_buf (int n, void *buf, size_t len)
{
  int data01 = !((BD_table[4*n+!ep[n].rx_odd].ctrl >> 6)&1);

  BD_table[4*n+ep[n].rx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6);
  BD_table[4*n+ep[n].rx_odd].buf = buf;
}

int
usb_lld_rx_data_len (int n)
{
  return (BD_table[4*n+!ep[n].rx_odd].ctrl >> 16)&0x3ff;
}


void
usb_lld_tx_enable_buf (int n, const void *buf, size_t len)
{
  int data01 = !((BD_table[4*n+2+!ep[n].tx_odd].ctrl >> 6)&1);

  BD_table[4*n+2+ep[n].tx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6); 
  BD_table[4*n+2+ep[n].tx_odd].buf = (void *)buf;
}
