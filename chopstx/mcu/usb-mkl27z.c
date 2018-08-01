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

#define DATA0 0
#define DATA1 1

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
static struct USB_CONF *const USB_CONF = (struct USB_CONF *)0x40072000;

struct USB_CTRL0 {
  volatile uint8_t OTGCTL;   /* OTG Control register                */
};
static struct USB_CTRL0 *const USB_CTRL0 = (struct USB_CTRL0 *)0x4007201c;

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
static struct USB_CTRL1 *const USB_CTRL1 = (struct USB_CTRL1 *)0x40072080;

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
static struct USB_ENDPT *const USB_ENDPT = (struct USB_ENDPT *)0x400720c0;

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
static struct USB_CTRL2 *const USB_CTRL2 = (struct USB_CTRL2 *)0x40072100;

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

static struct BD *const BD_table = (struct BD *)&__usb_bdt__;

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
kl27z_prepare_ep0_setup (struct usb_dev *dev)
{
  /* Endpoint 0, TX=0. */
  BD_table[ep[0].rx_odd].ctrl = 0x00080088; /* Len=8, OWN=1, DATA01=0, DTS=1 */
  BD_table[ep[0].rx_odd].buf = &dev->dev_req;

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

#include "usb_lld_driver.h"

static int handle_transaction (struct usb_dev *dev, uint8_t stat);

void
usb_lld_stall (int n)
{
  kl27z_ep_stall (n);
}


void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  dev->state = STALLED;
  kl27z_ep_stall (ENDP0);
}

int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  /* Zero length packet for ACK.  */
  dev->state = WAIT_STATUS_IN;
  kl27z_prepare_ep0_in (&dev->dev_req, 0, DATA1);
  return USB_EVENT_OK;
}


void
usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  kl27z_set_daddr (0);
  kl27z_usb_init ();

  /* Enable the endpoint 0.  */
  USB_ENDPT[0].EP = 0x0d;

  /* Clear Interrupt Status Register, and enable interrupt for USB */
  USB_CTRL1->ISTAT = 0xff;		     /* All clear */

  USB_CTRL1->INTEN = USB_IS_STALL | USB_IS_TOKDNE
                   | USB_IS_ERROR | USB_IS_USBRST;
}


#define USB_MAKE_EV(event) (event<<24)
#define USB_MAKE_TXRX(ep_num,txrx,len) ((txrx? (1<<23):0)|(ep_num<<16)|len)

int
usb_lld_event_handler (struct usb_dev *dev)
{
  uint8_t istat_value = USB_CTRL1->ISTAT;
  uint8_t stat = USB_CTRL1->STAT;

  if ((istat_value & USB_IS_USBRST))
    {
      USB_CTRL1->ISTAT = USB_IS_USBRST;
      return USB_MAKE_EV (USB_EVENT_DEVICE_RESET);
    }
  else if ((istat_value & USB_IS_TOKDNE))
    return handle_transaction (dev, stat);
  else if ((istat_value & USB_IS_ERROR))
    { /* Clear Errors.  */
      USB_CTRL1->ERRSTAT = USB_CTRL1->ERRSTAT;
      USB_CTRL1->ISTAT = USB_IS_ERROR;
    }
  else if ((istat_value & USB_IS_STALL))
    {
      /* Does STAT have ENDPOINT info in this case?: No, it doesn't.  */

      if (kl27z_ep_is_stall (0))
	{ /* It's endpoint 0, recover from erorr.  */
	  dev->state = WAIT_SETUP;
	  kl27z_ep_clear_stall (0);
	  kl27z_prepare_ep0_setup (dev);
	}

      USB_CTRL1->ISTAT = USB_IS_STALL;
    }

  return USB_EVENT_OK;
}


static void
handle_datastage_out (struct usb_dev *dev, uint8_t stat)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
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
      dev->state = WAIT_STATUS_IN;
      kl27z_prepare_ep0_in (&dev->dev_req, 0, DATA1);
    }
  else
    {
      dev->state = OUT_DATA;
      kl27z_prepare_ep0_out (data_p->addr, len, data01);
    }
}

static void
handle_datastage_in (struct usb_dev *dev, uint8_t stat)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  int odd = (stat >> 2)&1;
  int data01 = !((BD_table[2+odd].ctrl >> 6)&1);
  uint32_t len = USB_MAX_PACKET_SIZE;

  if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  kl27z_prepare_ep0_in (&dev->dev_req, 0, data01);
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.  */
	  dev->state = WAIT_STATUS_OUT;
	  kl27z_prepare_ep0_out (&dev->dev_req, 0, DATA1);
	}

      return;
    }

  dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  kl27z_prepare_ep0_in (data_p->addr, len, data01);
  data_p->len -= len;
  data_p->addr += len;
}

typedef int (*HANDLER) (struct usb_dev *dev);

static int
std_none (struct usb_dev *dev)
{
  (void)dev;
  return -1;
}

static uint16_t status_info;

static int
std_get_status (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (arg->value != 0 || arg->len != 2 || (arg->index >> 8) != 0
      || USB_SETUP_SET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->index == 0)
	{
	  /* Get Device Status */
	  uint8_t feature = dev->feature;

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

	  return usb_lld_ctrl_send (dev, &status_info, 2);
	}
    }
  else if (rcp == INTERFACE_RECIPIENT)
    {
      if (dev->configuration == 0)
	return -1;

      return USB_EVENT_GET_STATUS_INTERFACE;
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if ((arg->index & 0x70) || n == ENDP0)
	return -1;

      if (kl27z_ep_is_disabled (n))
	return -1;

      status_info = kl27z_ep_is_stall (n);
      return usb_lld_ctrl_send (dev, &status_info, 2);
    }

  return -1;
}

static int
std_clear_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature &= ~(1 << 5);
	  return USB_EVENT_CLEAR_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || n == ENDP0)
	return -1;

      if (kl27z_ep_is_disabled (n))
	return -1;

      kl27z_ep_clear_dtog ((arg->index & 0x80) == 0, n);

      return USB_EVENT_CLEAR_FEATURE_ENDPOINT;
    }

  return -1;
}

static int
std_set_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature |= 1 << 5;
	  return USB_EVENT_SET_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || n == ENDP0)
	return -1;

      if (kl27z_ep_is_disabled (n))
	return -1;

      kl27z_ep_stall (n);

      return USB_EVENT_SET_FEATURE_ENDPOINT;
    }

  return -1;
}

static int
std_set_address (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT && arg->len == 0 && arg->value <= 127
      && arg->index == 0 && dev->configuration == 0)
    return usb_lld_ctrl_ack (dev);

  return -1;
}

static int
std_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  if (USB_SETUP_SET (arg->type))
    return -1;

  return USB_EVENT_GET_DESCRIPTOR;
}

static int
std_get_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || arg->index != 0 || arg->len != 1)
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    return usb_lld_ctrl_send (dev, &dev->configuration, 1);

  return -1;
}

static int
std_set_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT && arg->index == 0 && arg->len == 0)
    return USB_EVENT_SET_CONFIGURATION;

  return -1;
}

static int
std_get_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || (arg->index >> 8) != 0 || arg->len != 1)
    return -1;

  if (dev->configuration == 0)
    return -1;

  if (rcp == INTERFACE_RECIPIENT)
    return USB_EVENT_GET_INTERFACE;

  return -1;
}

static int
std_set_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev->configuration == 0)
    return -1;

  return USB_EVENT_SET_INTERFACE;
}


static int
handle_setup0 (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  int r = -1;
  HANDLER handler;

  data_p->addr = NULL;
  data_p->len = 0;
  data_p->require_zlp = 0;

  if ((dev->dev_req.type & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      switch (dev->dev_req.request)
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

      if ((r = (*handler) (dev)) < 0)
	{
	  usb_lld_ctrl_error (dev);
	  return USB_EVENT_OK;
	}
      else
	return r;
    }
  else
    return USB_EVENT_CTRL_REQUEST;
}

static int
handle_in0 (struct usb_dev *dev, uint8_t stat)
{
  int r = 0;

  if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    handle_datastage_in (dev, stat);
  else if (dev->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
      uint16_t value = dev->dev_req.value;

      if ((dev->dev_req.request == SET_ADDRESS) &&
	  ((dev->dev_req.type & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	{
	  kl27z_set_daddr (value);
	  ep[0].rx_odd = 0;
	  r = USB_EVENT_DEVICE_ADDRESSED;
	}
      else
	r = USB_EVENT_CTRL_WRITE_FINISH;
      dev->state = WAIT_SETUP;
      kl27z_prepare_ep0_setup (dev);
    }
  else
    {
      dev->state = STALLED;
      kl27z_ep_stall (ENDP0);
    }

  return r;
}

static void
handle_out0 (struct usb_dev *dev, uint8_t stat)
{
  if (dev->state == OUT_DATA)
    /* It's normal control WRITE transfer.  */
    handle_datastage_out (dev, stat);
  else if (dev->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
      dev->state = WAIT_SETUP;
      kl27z_prepare_ep0_setup (dev);
    }
  else
    {
      /*
       * dev->state == IN_DATA || dev->state == LAST_IN_DATA
       * (Host aborts the transfer before finish)
       * Or else, unexpected state.
       * STALL the endpoint, until we receive the next SETUP token.
       */
      dev->state = STALLED;
      kl27z_ep_stall (ENDP0);
    }
}

#define USB_TOKEN_ACK   0x02
#define USB_TOKEN_IN    0x09
#define USB_TOKEN_SETUP 0x0d

static int
handle_transaction (struct usb_dev *dev, uint8_t stat)
{
  int odd = (stat >> 2)&1;
  uint8_t ep_num = (stat >> 4);
  int r;

  if (ep_num == 0)
    {
      if ((stat & 0x08) == 0)
	{
	  ep[0].rx_odd ^= 1;
	  if (TOK_PID (BD_table[odd].ctrl) == USB_TOKEN_SETUP)
	    {
	      r = handle_setup0 (dev);
	      USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	      USB_CTRL1->CTL = 0x01; /* Clear TXSUSPENDTOKENBUSY.  */
	      return USB_MAKE_EV (r);
	    }
	  else
	    {
	      handle_out0 (dev, stat);
	      USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	      return USB_EVENT_OK;
	    }
	}
      else
	{
	  ep[0].tx_odd ^= 1;
	  r = handle_in0 (dev, stat);
	  USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	  return USB_MAKE_EV (r);
	}
    }
  else
    {
      uint16_t len;

      if ((stat & 0x08) == 0)
	{
	  len = (BD_table[4*ep_num+odd].ctrl >> 16)&0x3ff;
	  ep[ep_num].rx_odd ^= 1;
	  USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	  return USB_MAKE_TXRX (ep_num, 0, len);
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
	   * Although it is not described in the specification (it's
	   * ambiguous), it is actually possible for some host
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

	  len = (BD_table[4*ep_num+2+odd].ctrl >> 16)&0x3ff;
	  if (!success)
	    USB_CTRL1->ERRSTAT = dmaerr; /* Clear error.  */

	  ep[ep_num].tx_odd ^= 1;
	  USB_CTRL1->ISTAT = USB_IS_TOKDNE;
	  return USB_MAKE_TXRX (ep_num, 1, len);
	}
    }
}

void
usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  /* Reset USB */
  USB_CTRL2->USBTRC0 = 0xc0;

  USB_CTRL1->CTL = 0x00;    /* Disable USB FS communication module */

  kl27z_set_daddr (0);
  kl27z_usb_init ();

  /* Clear Interrupt Status Register, and enable interrupt for USB */
  USB_CTRL1->ISTAT = 0xff;		     /* All clear */

  USB_CTRL1->INTEN = USB_IS_STALL | USB_IS_TOKDNE
                   | USB_IS_ERROR | USB_IS_USBRST;
}

void
usb_lld_setup_endp (struct usb_dev *dev, int n, int rx_en, int tx_en)
{
  if (n == 0)
    {
      /* Enable the endpoint 0.  */
      USB_ENDPT[0].EP = 0x0d;
      kl27z_prepare_ep0_setup (dev);
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
usb_lld_set_configuration (struct usb_dev *dev, uint8_t config)
{
  dev->configuration = config;
}

uint8_t
usb_lld_current_configuration (struct usb_dev *dev)
{
  return dev->configuration;
}

int
usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  data_p->addr = (uint8_t *)p;
  data_p->len = len;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;

  kl27z_prepare_ep0_out (p, len, DATA1);
  dev->state = OUT_DATA;
  return USB_EVENT_OK;
}

/*
 * BUF: Pointer to data memory.  Data memory should not be allocated
 *      on stack when BUFLEN > USB_MAX_PACKET_SIZE.
 *
 * BUFLEN: size of the data.
 */
int
usb_lld_ctrl_send (struct usb_dev *dev, const void *buf, size_t buflen)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len_asked = dev->dev_req.len;
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
      dev->state = LAST_IN_DATA;
    }
  else
    {
      len = USB_MAX_PACKET_SIZE;
      dev->state = IN_DATA;
    }

  kl27z_prepare_ep0_in (data_p->addr, len, DATA1);

  data_p->len -= len;
  data_p->addr += len;

  return USB_EVENT_OK;
}

void
usb_lld_rx_enable_buf (int n, void *buf, size_t len)
{
  int data01 = !((BD_table[4*n+!ep[n].rx_odd].ctrl >> 6)&1);

  BD_table[4*n+ep[n].rx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6);
  BD_table[4*n+ep[n].rx_odd].buf = buf;
}


void
usb_lld_tx_enable_buf (int n, const void *buf, size_t len)
{
  int data01 = !((BD_table[4*n+2+!ep[n].tx_odd].ctrl >> 6)&1);

  BD_table[4*n+2+ep[n].tx_odd].ctrl = (len << 16) | 0x0088 | (data01 << 6); 
  BD_table[4*n+2+ep[n].tx_odd].buf = (void *)buf;
}
