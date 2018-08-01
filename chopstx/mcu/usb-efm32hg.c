/*
 * usb-efm32.c - USB driver for EFM32HG
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of Chpostx port to EFM32HG
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
#include <string.h>

#include "usb_lld.h"
#include "usb_lld_driver.h"

#include "efm32hg.h"

#define DEPCTL_WO_MASK 0x3c000000UL
#define DCTL_WO_MASK   0x780UL

/* Implementation */

static struct device_req ep0_setup_pkt[3] __attribute__ ((aligned(4)));
static char ctrl_send_buf[USB_MAX_PACKET_SIZE] __attribute__ ((aligned(4)));

/* Enpoint state struct */
struct efm32hg_ep
{
  uint32_t xfersize;
};

/* Output endpoints */
static struct efm32hg_ep oeps[4];

static void
efm32hg_ep_out_stall (uint8_t n)
{
  uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_MASK;
  uint32_t eptype = ctl & 0xC0000UL;

  if (eptype != USB_DOEP_CTL_EPTYPE_ISO)
    {
      ctl |= USB_DIEP_CTL_STALL;
      USB_DOUTEPS[n].CTL = ctl;
    }
}

static void
efm32hg_ep_in_stall (uint8_t n)
{
  uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_MASK;
  uint32_t eptype = ctl & 0xC0000UL;

  if (eptype != USB_DIEP_CTL_EPTYPE_ISO)
    {
      USB_DINEPS[n].CTL |= USB_DIEP_CTL_STALL;
      if (ctl & USB_DIEP_CTL_EPENA)
        ctl |= USB_DIEP_CTL_EPDIS;
      USB_DINEPS[n].CTL = ctl;
    }
}

static void
efm32hg_ep_out_unstall (uint8_t n)
{
  uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_MASK;
  uint32_t eptype = ctl & 0xC0000UL;

  if (eptype == USB_DOEP_CTL_EPTYPE_INT || eptype == USB_DOEP_CTL_EPTYPE_BULK)
    {
      ctl |= USB_DOEP_CTL_SETD0PIDEF;
      ctl &= ~USB_DOEP_CTL_STALL;
      USB_DOUTEPS[n].CTL = ctl;
    }
}

static void
efm32hg_ep_in_unstall (uint8_t n)
{
  uint32_t ctl = USB_DINEPS[n].CTL & ~DEPCTL_WO_MASK;
  uint32_t eptype = ctl & 0xC0000UL;

  if (eptype == USB_DIEP_CTL_EPTYPE_INT || eptype == USB_DIEP_CTL_EPTYPE_BULK)
    {
      ctl |= USB_DIEP_CTL_SETD0PIDEF;
      ctl &= ~USB_DIEP_CTL_STALL;
      USB_DINEPS[n].CTL = ctl;
    }
}

static int
efm32hg_ep_in_is_stall (uint8_t n)
{
  return (USB_DINEPS[n].CTL & USB_DIEP_CTL_STALL) ? 1 : 0;
}

static int
efm32hg_ep_out_is_stall (uint8_t n)
{
  return (USB_DOUTEPS[n].CTL & USB_DOEP_CTL_STALL) ? 1 : 0;
}

static int
efm32hg_ep_in_is_disabled (uint8_t n)
{
  return (USB_DINEPS[n].CTL & USB_DIEP_CTL_EPENA) ? 0 : 1;
}

static int
efm32hg_ep_out_is_disabled (uint8_t n)
{
  return (USB_DOUTEPS[n].CTL & USB_DOEP_CTL_EPENA) ? 0 : 1;
}

static void
efm32hg_set_daddr (uint8_t daddr)
{
  USB->DCFG = (USB->DCFG & ~USB_DCFG_DEVADDR_MASK) | (daddr << 4);
}

static void
efm32hg_enable_ints (void)
{
  /* Disable all interrupts. */
  USB->GINTMSK = 0;

  /* Clear pending interrupts */
  USB->GINTSTS = 0xFFFFFFFF;

  USB->GINTMSK = USB_GINTMSK_USBRSTMSK
                 | USB_GINTMSK_ENUMDONEMSK
                 | USB_GINTMSK_IEPINTMSK
                 | USB_GINTMSK_OEPINTMSK;
}

static void
efm32hg_disable_ep_in (uint8_t n)
{
  USB_DINEPS[n].CTL = 0;
  USB->DAINTMSK &= ~(1 << n);
}

static void
efm32hg_disable_ep_out (uint8_t n)
{
  USB_DINEPS[n].CTL = 0;
  USB->DAINTMSK &= ~(1 << (n + 16));
}

static void
efm32hg_prepare_ep0_setup (struct usb_dev *dev)
{
  (void) dev;

  USB->DOEP0TSIZ = (8*3 <<  0)  /* XFERSIZE */
                   | (1 << 19)  /* PKTCNT */
                   | (3 << 29); /* SUPCNT */
  USB->DOEP0DMAADDR = (uint32_t) &ep0_setup_pkt;
  USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_MASK) | USB_DOEP0CTL_EPENA;
}

static void
efm32hg_prepare_ep0_out (const void *buf, size_t len, uint32_t ep0mps)
{
  USB->DOEP0DMAADDR = (uint32_t) buf;
  USB->DOEP0TSIZ = (len <<  0)  /* XFERSIZE */
                   | (1 << 19); /* PKTCNT */
  USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_MASK)
                  | USB_DOEP0CTL_CNAK | USB_DOEP0CTL_EPENA
                  | ep0mps;
}

static void
efm32hg_prepare_ep0_in (const void *buf, size_t len, uint32_t ep0mps)
{
  USB->DIEP0DMAADDR = (uint32_t) buf;
  USB->DIEP0TSIZ = (len <<  0)  /* XFERSIZE */
                   | (1 << 19); /* PKTCNT */
  USB->DIEP0CTL = (USB->DIEP0CTL & ~DEPCTL_WO_MASK)
                  | USB_DIEP0CTL_CNAK | USB_DIEP0CTL_EPENA
                  | ep0mps;
}

static void
handle_datastage_out (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len = USB->DOEP0TSIZ & 0x7FUL; /* XFERSIZE */
  uint32_t pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);

  data_p->len -= len;
  data_p->addr += len;

  len = data_p->len < pktsize ? data_p->len : pktsize;

  if (data_p->len == 0)
    {
      /* No more data to receive, proceed to send acknowledge for IN.  */
      efm32hg_prepare_ep0_setup (dev);
      dev->state = WAIT_STATUS_IN;
      efm32hg_prepare_ep0_in (NULL, 0, 0);
    }
  else
    {
      dev->state = OUT_DATA;
      efm32hg_prepare_ep0_out (data_p->addr, len, 0);
    }
}

static void
handle_datastage_in (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len = 64 >> (USB->DOEP0CTL & 0x3UL);

  if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
        {
          data_p->require_zlp = 0;

          efm32hg_prepare_ep0_setup (dev);
          /* No more data to send.  Send empty packet */
          efm32hg_prepare_ep0_in (NULL, 0, 0);
        }
      else
        {
          /* No more data to send, proceed to receive OUT acknowledge.  */
          dev->state = WAIT_STATUS_OUT;
          efm32hg_prepare_ep0_out (NULL, 0, 0);
        }

      return;
    }

  dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  efm32hg_prepare_ep0_setup (dev);
  efm32hg_prepare_ep0_in (data_p->addr, len, 0);
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

static uint16_t status_info __attribute__((aligned(4)));

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


      if (arg->index & 0x80)
        {
          /* IN endpoint */

          if (efm32hg_ep_in_is_disabled (n))
            return -1;

          status_info = efm32hg_ep_in_is_stall (n);
        }
      else
        {
          /* OUT endpoint */

          if (efm32hg_ep_out_is_disabled (n))
            return -1;

          status_info = efm32hg_ep_out_is_stall (n);
        }

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


      if (arg->index & 0x80)
        {
          /* IN endpoint */

          if (efm32hg_ep_in_is_disabled (n))
            return -1;

          efm32hg_ep_in_unstall (n);
        }
      else
        {
          /* OUT endpoint */

          if (efm32hg_ep_out_is_disabled (n))
            return -1;

          efm32hg_ep_out_unstall (n);
        }

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

      if (arg->index & 0x80)
        {
          /* IN endpoint */

          if (efm32hg_ep_in_is_disabled (n))
            return -1;

          efm32hg_ep_in_stall (n);
        }
      else
        {
          /* OUT endpoint */

          if (efm32hg_ep_out_is_disabled (n))
            return -1;

          efm32hg_ep_out_stall (n);
        }

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
    {
      efm32hg_set_daddr (arg->value);
      efm32hg_prepare_ep0_setup (dev);
      dev->state = WAIT_STATUS_IN;
      efm32hg_prepare_ep0_in (NULL, 0, 0);
      return USB_EVENT_DEVICE_ADDRESSED;
    }

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
handle_in0 (struct usb_dev *dev)
{
  if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    handle_datastage_in (dev);
  else if (dev->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
      efm32hg_prepare_ep0_setup (dev);
      dev->state = WAIT_SETUP;
      return USB_EVENT_CTRL_WRITE_FINISH;
    }
  else
    {
      dev->state = STALLED;
      efm32hg_ep_out_stall (ENDP0);
      efm32hg_ep_in_stall (ENDP0);
      dev->state = WAIT_SETUP;
      efm32hg_prepare_ep0_setup (dev);
    }

  return USB_EVENT_OK;
}

static int
handle_out0 (struct usb_dev *dev)
{
  if (dev->state == OUT_DATA)
    /* It's normal control WRITE transfer.  */
    handle_datastage_out (dev);
  else if (dev->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
      efm32hg_prepare_ep0_setup (dev);
      dev->state = WAIT_SETUP;
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
      efm32hg_ep_out_stall (ENDP0);
      efm32hg_ep_in_stall (ENDP0);
      dev->state = WAIT_SETUP;
      efm32hg_prepare_ep0_setup (dev);
    }
    return USB_EVENT_OK;
}


struct efm32hg_rev
{
  uint8_t family;
  uint8_t minor;
  uint8_t major;
};

void
efm32hg_revno (struct efm32hg_rev *rev)
{
  uint8_t tmp;

  /* CHIP FAMILY bit [5:2] */
  tmp  = ((ROMTABLE->PID1 & 0xf) << 2);
  /* CHIP FAMILY bit [1:0] */
  tmp |=  ((ROMTABLE->PID0 & 0xc0) >> 6);
  rev->family = tmp;
 
  /* CHIP MAJOR bit [3:0] */
  rev->major = (ROMTABLE->PID0 & 0x3f);
 
  /* CHIP MINOR bit [7:4] */
  tmp  = (((ROMTABLE->PID2 & 0xf0) >> 4) << 4);
  /* CHIP MINOR bit [3:0] */
  tmp |=  ((ROMTABLE->PID3 & 0xf0) >> 4);
  rev->minor = tmp;
}

void
efm32hg_usb_init (void)
{
  struct efm32hg_rev rev;

  /* Ensure selected oscillator is enabled, waiting for it to stabilize */
  CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
  while (!(CMU->STATUS & CMU_STATUS_LFRCORDY));

  /* Select LFRCO as LFCCLK clock */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~0x30UL) | (0x1 /* LFRCO */ << 4 /* LFC */);

  CMU->LFCCLKEN0 |= CMU_LFCCLKEN0_USBLE;

  /* Enable USB clock */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_USB | CMU_HFCORECLKEN0_USBC;

  CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_48MHZ;

  /* Select USHFRCO as clock source for USB */
  CMU->OSCENCMD = CMU_OSCENCMD_USHFRCOEN;
  while (!(CMU->STATUS & CMU_STATUS_USHFRCORDY));

  /* Switch oscillator */
  CMU->CMD = CMU_CMD_USBCCLKSEL_USHFRCO;

  /* Wait until clock is activated */
  while ((CMU->STATUS & CMU_STATUS_USBCUSHFRCOSEL) == 0);

  /* Enable USHFRCO Clock Recovery mode. */
  CMU->USBCRCTRL |= CMU_USBCRCTRL_EN;

  /* Turn on Low Energy Mode (LEM) features. */
  efm32hg_revno (&rev);

  if ((rev.family == 5) && (rev.major == 1) && (rev.minor == 0))
    {
      /* First Happy Gecko chip revision did not have 
      all LEM features enabled. */
      USB->CTRL = USB_CTRL_LEMOSCCTRL_GATE
                  | USB_CTRL_LEMIDLEEN
                  | USB_CTRL_LEMPHYCTRL;
    }
  else
    {
      USB->CTRL = USB_CTRL_LEMOSCCTRL_GATE
                  | USB_CTRL_LEMIDLEEN
                  | USB_CTRL_LEMPHYCTRL
                  | USB_CTRL_LEMNAKEN
                  | USB_CTRL_LEMADDRMEN;
    }

}

static void
efm32hg_connect (void)
{
  USB->DCTL &= ~(DCTL_WO_MASK | USB_DCTL_SFTDISCON);
}

static void __attribute__((unused))
efm32hg_disconnect (void)
{
  USB->DCTL = (USB->DCTL & ~ DCTL_WO_MASK) | USB_DCTL_SFTDISCON;
}

static void
efm32hg_flush_rx_fifo (void)
{
  USB->GRSTCTL = USB_GRSTCTL_RXFFLSH;
  while (USB->GRSTCTL & USB_GRSTCTL_RXFFLSH) {}
}

static void
efm32hg_flush_tx_fifo (uint8_t n)
{
  USB->GRSTCTL = USB_GRSTCTL_TXFFLSH | (n << 6);
  while (USB->GRSTCTL & USB_GRSTCTL_TXFFLSH) {}
}

void
efm32hg_core_reset (void)
{
  USB->PCGCCTL &= ~USB_PCGCCTL_STOPPCLK;
  USB->PCGCCTL &= ~(USB_PCGCCTL_PWRCLMP | USB_PCGCCTL_RSTPDWNMODULE);

  /* Core Soft Reset */
  USB->GRSTCTL |= USB_GRSTCTL_CSFTRST;
  while (USB->GRSTCTL & USB_GRSTCTL_CSFTRST) {}

  /* Wait for AHB master IDLE state. */
  while (!(USB->GRSTCTL & USB_GRSTCTL_AHBIDLE)) {}
}

int
efm32hg_core_init (void)
{
  const uint32_t total_rx_fifo_size = 128;
  const uint32_t total_tx_fifo_size = 256;
  const uint32_t ep_tx_fifo_size = 64;
  uint32_t address, depth;
  uint8_t ep;

  USB->ROUTE = USB_ROUTE_PHYPEN; /* Enable PHY pins.  */

  efm32hg_core_reset ();

  /* Setup full speed device */
  USB->DCFG = (USB->DCFG & ~USB_DCFG_DEVSPD_MASK) | USB_DCFG_DEVSPD_FS;

  /* Stall on non-zero len status OUT packets (ctrl transfers). */
  USB->DCFG |= USB_DCFG_NZSTSOUTHSHK;

  /* Set periodic frame interval to 80% */
  USB->DCFG &= ~USB_DCFG_PERFRINT_MASK;

  USB->GAHBCFG = (USB->GAHBCFG & ~USB_GAHBCFG_HBSTLEN_MASK)
                 | USB_GAHBCFG_DMAEN | USB_GAHBCFG_HBSTLEN_SINGLE;

  /* Ignore frame numbers on ISO transfers. */
  USB->DCTL = (USB->DCTL & ~DCTL_WO_MASK) | USB_DCTL_IGNRFRMNUM;

  /* Set Rx FIFO size */
  USB->GRXFSIZ = total_rx_fifo_size;

  /* Set Tx EP0 FIFO size */
  address = total_rx_fifo_size;
  depth = ep_tx_fifo_size;
  USB->GNPTXFSIZ = (depth << 16 /*NPTXFINEPTXF0DEP*/) | address /*NPTXFSTADDR*/;

  /* Set Tx EP FIFO sizes for all IN ep's */
  for (ep = 1; ep <= MAX_NUM_IN_EPS; ep++)
    {
      address += depth;
      depth = ep_tx_fifo_size;
      USB_DIEPTXFS[ep - 1] = (depth << 16)           /* INEPNTXFDEP */
                             | (address & 0x7FFUL);  /* INEPNTXFSTADDR */
    }

  if (total_rx_fifo_size + total_tx_fifo_size > MAX_DEVICE_FIFO_SIZE_INWORDS)
    return -1;

  if (address > MAX_DEVICE_FIFO_SIZE_INWORDS)
    return -1;

  /* Flush the FIFO's */
  efm32hg_flush_tx_fifo (0x10);        /* All Tx FIFO's */
  efm32hg_flush_rx_fifo ();            /* The Rx FIFO   */

  /* Disable all device interrupts */
  USB->DIEPMSK    = 0;
  USB->DOEPMSK    = 0;
  USB->DAINTMSK   = 0;
  USB->DIEPEMPMSK = 0;

  /* Disable all EP's, clear all EP ints. */
  for (ep = 0; ep <= MAX_NUM_IN_EPS; ep++)
    {
      USB_DINEPS[ep].CTL  = 0;
      USB_DINEPS[ep].TSIZ = 0;
      USB_DINEPS[ep].INT  = 0xFFFFFFFF;
    }

  for (ep = 0; ep <= MAX_NUM_OUT_EPS; ep++)
    {
      USB_DOUTEPS[ep].CTL  = 0;
      USB_DOUTEPS[ep].TSIZ = 0;
      USB_DOUTEPS[ep].INT  = 0xFFFFFFFF;
    }

  efm32hg_connect ();

  return 0;
}

/* INTERFACE */

void
usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  efm32hg_usb_init ();

  efm32hg_core_init ();

  efm32hg_set_daddr (0);

  /* Unmask interrupts for TX and RX */
  USB->GAHBCFG |= USB_GAHBCFG_GLBLINTRMSK;
  USB->GINTMSK = /*USB_GINTMSK_USBSUSPMSK
                 |*/ USB_GINTMSK_USBRSTMSK
                 | USB_GINTMSK_ENUMDONEMSK
                 | USB_GINTMSK_IEPINTMSK
                 | USB_GINTMSK_OEPINTMSK
                 /*| USB_GINTMSK_WKUPINTMSK*/;
  USB->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
  USB->DOEPMSK  = USB_DOEPMSK_SETUPMSK | USB_DOEPMSK_XFERCOMPLMSK
                  | USB_DOEPMSK_STSPHSERCVDMSK;
  USB->DIEPMSK  = USB_DIEPMSK_XFERCOMPLMSK;
}


#define USB_MAKE_EV(event) (event<<24)
#define USB_MAKE_TXRX(ep_num,txrx,len) ((txrx? (1<<23):0)|(ep_num<<16)|len)

int
usb_lld_event_handler (struct usb_dev *dev)
{
  uint32_t intsts = USB->GINTSTS & USB->GINTMSK;
  uint16_t len;
  uint8_t ep;
  int r = USB_EVENT_OK;

  if (intsts & USB_GINTSTS_USBRST)
    {
      USB->GINTSTS = USB_GINTSTS_USBRST;
      return USB_MAKE_EV (USB_EVENT_DEVICE_RESET);
    }

  if (intsts & USB_GINTSTS_ENUMDONE)
    {
      USB->GINTSTS = USB_GINTSTS_ENUMDONE;
      efm32hg_prepare_ep0_setup (dev);
      efm32hg_enable_ints ();
      dev->state = WAIT_SETUP;
    }

  if (intsts & USB_GINTSTS_IEPINT)
    {
      uint32_t epint = USB->DAINT & USB->DAINTMSK;

      ep = 0;
      while (epint != 0)
        {
          if (epint & 1)
            {
              uint32_t sts = USB_DINEPS[ep].INT & USB->DIEPMSK;
              if (sts & USB_DIEP_INT_XFERCOMPL)
                {
                  USB_DINEPS[ep].INT = USB_DIEP_INT_XFERCOMPL;
                  if (ep == 0)
                    r = handle_in0 (dev);
                  else
                    {
                      len = USB_DINEPS[ep].TSIZ & 0x7FFFFUL; /* XFERSIZE */
                      if (USB_DINEPS[ep].INT & USB_DIEP_INT_NAKINTRPT)
                        USB_DINEPS[ep].INT = USB_DIEP_INT_NAKINTRPT;
                      return USB_MAKE_TXRX (ep, 1, len);
                    }
                }
            }
          ep++;
          epint >>= 1;
        }
    }

  if (intsts & USB_GINTSTS_OEPINT)
    {
      uint32_t epint = (USB->DAINT & USB->DAINTMSK) >> 16;

      ep = 0;
      while (epint != 0)
        {
          if (epint & 1)
            {
              uint32_t sts = USB_DOUTEPS[ep].INT & USB->DOEPMSK;

              if (ep == 0 && sts & USB_DOEP0INT_STUPPKTRCVD)
                {
                  USB_DOUTEPS[ep].INT = USB_DOEP0INT_STUPPKTRCVD;
                  sts &= ~USB_DOEP_INT_XFERCOMPL;
                }

              if (sts & USB_DOEP_INT_XFERCOMPL)
                {
                  USB_DOUTEPS[ep].INT = USB_DOEP_INT_XFERCOMPL;
                  if (ep == 0)
                    {
                      sts = USB->DOEP0INT & USB->DOEPMSK;

                      USB_DOUTEPS[ep].INT = USB_DOEP0INT_STUPPKTRCVD;

                      if (sts & USB_DOEP0INT_SETUP)
                        {
                          USB->DOEP0INT = USB_DOEP0INT_SETUP;
                          int supcnt = (USB->DOEP0TSIZ & 0x60000000UL) >> 29;
                          supcnt = (supcnt == 3) ? 2 : supcnt;
                          dev->dev_req = ep0_setup_pkt[2 - supcnt];
                          r = handle_setup0 (dev);
                        }
                      else if (dev->state != WAIT_SETUP)
                        r = handle_out0 (dev);
                    }
                  else /* ep != 0 */
                    {
                      uint32_t remain = USB_DOUTEPS[ep].TSIZ & 0x7FFFFUL; /* XFERSIZE */
                      if (remain < oeps[ep].xfersize)
                        len = oeps[ep].xfersize - remain;
                      else
                        len = 0;
                      return USB_MAKE_TXRX (ep, 0, len);
                    }
                }
              else
                {
                  if (ep == 0 && sts & USB_DOEP0INT_SETUP)
                    {
                      USB->DOEP0INT = USB_DOEP0INT_SETUP;
                      int supcnt = (USB->DOEP0TSIZ & 0x60000000UL) >> 29;
                      supcnt = (supcnt == 3) ? 2 : supcnt;
                      dev->dev_req = ep0_setup_pkt[2 - supcnt];
                      r = handle_setup0 (dev);
                    }
                }

              if (sts & USB_DOEP0INT_STSPHSERCVD)
                USB->DOEP0INT = USB_DOEP0INT_STSPHSERCVD;
            }
          ep++;
          epint >>= 1;
        }
    }

  return USB_MAKE_EV (r);
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
  uint32_t pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);

  data_p->addr = (void *)buf;
  data_p->len = buflen;

  /* Restrict the data length to be the one host asks for */
  if (data_p->len > len_asked)
    data_p->len = len_asked;

  data_p->require_zlp = (data_p->len != 0 && (data_p->len % pktsize) == 0);

  if (((uint32_t) data_p->addr & 3) && (data_p->len <= pktsize))
    {
      data_p->addr = (void *) ctrl_send_buf;
      memcpy (data_p->addr, buf, buflen);
    }

  if (data_p->len < pktsize)
    {
      len = data_p->len;
      dev->state = LAST_IN_DATA;
    }
  else
    {
      len = pktsize;
      dev->state = IN_DATA;
    }

  efm32hg_prepare_ep0_in (data_p->addr, len, 0);

  data_p->len -= len;
  data_p->addr += len;

  return USB_EVENT_OK;
}

int
usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);
  data_p->addr = (uint8_t *)p;
  data_p->len = len;
  if (len > pktsize)
    len = pktsize;

  efm32hg_prepare_ep0_out (p, len, 0);
  dev->state = OUT_DATA;
  return USB_EVENT_OK;
}

int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  /* Zero length packet for ACK.  */
  efm32hg_prepare_ep0_setup (dev);
  dev->state = WAIT_STATUS_IN;
  efm32hg_prepare_ep0_in (NULL, 0, 0);
  return USB_EVENT_OK;
}

void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  dev->state = STALLED;
  efm32hg_ep_out_stall (ENDP0);
  efm32hg_ep_in_stall (ENDP0);
  dev->state = WAIT_SETUP;
  efm32hg_prepare_ep0_setup (dev);
}

#define NVIC_ICPR ((uint32_t *)(0xe000e280))
#define NVIC_INTR_CLR(n)  { NVIC_ICPR[n >> 5] = 1 << (n & 0x1f); }

void
usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  int i;

  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  /* Clear Remote Wakeup Signalling */
  USB->DCTL &= ~(DCTL_WO_MASK | USB_DCTL_RMTWKUPSIG);
  efm32hg_flush_tx_fifo (0);

  /* Clear pending interrupts */
  for (i = 0; i <= MAX_NUM_IN_EPS; i++)
    USB_DINEPS[i].INT = 0xFFFFFFFF;

  for (i = 0; i <= MAX_NUM_OUT_EPS; i++)
    USB_DOUTEPS[i].INT = 0xFFFFFFFF;

  USB->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
  USB->DOEPMSK = USB_DOEPMSK_SETUPMSK | USB_DOEPMSK_XFERCOMPLMSK
                | USB_DOEPMSK_STSPHSERCVDMSK;
  USB->DIEPMSK = USB_DIEPMSK_XFERCOMPLMSK;

  /* Reset Device Address */
  efm32hg_set_daddr (0);

  /* Setup EP0 to receive SETUP packets */
  efm32hg_prepare_ep0_setup (dev);
  efm32hg_enable_ints ();

  for (i = 0; i < MAX_NUM_IN_EPS; ++i)
    efm32hg_disable_ep_in (i);

  for (i = 0; i < MAX_NUM_OUT_EPS; ++i)
    efm32hg_disable_ep_out (i);

  /* Clear pending interrupts for IRQ line 19 hack. chx_clr_intr isn't exposed
  and I don't feel like it needs to be. */
  NVIC_INTR_CLR (19);
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

void
usb_lld_tx_enable_buf (int ep_num, const void *buf, size_t len)
{
  uint32_t pktsize;
  uint32_t pktcnt, xfersize;

  if (ep_num == 0)
    pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);
  else
    pktsize = USB_DINEPS[ep_num].CTL & 0x7FFUL;

  if (len == 0)
    {
      /* zlp */
      pktcnt = 1;
      xfersize = 0;
    }
  else
    {
      pktcnt = (len - 1 + pktsize) / pktsize;
      xfersize = len;
    }

  USB_DINEPS[ep_num].TSIZ = (USB_DINEPS[ep_num].TSIZ & ~0x7fffffff)
                            | (xfersize << 0)
                            | (pktcnt << 19);
  USB_DINEPS[ep_num].DMAADDR = (uint32_t) buf;
  USB_DINEPS[ep_num].CTL = (USB_DINEPS[ep_num].CTL & ~DEPCTL_WO_MASK)
                           | USB_DIEP_CTL_CNAK
                           | USB_DIEP_CTL_EPENA;
}

void
usb_lld_rx_enable_buf (int ep_num, void *buf, size_t len)
{
  uint32_t pktsize;
  uint32_t pktcnt, xfersize;

  if (ep_num == 0)
    pktsize = 64 >> (USB->DOEP0CTL & 0x3UL);
  else
    pktsize = USB_DOUTEPS[ep_num].CTL & 0x7FFUL;

  if (len == 0)
    {
      /* zlp */
      pktcnt = 1;
      xfersize = 0;
    }
  else
    {
      pktcnt = (len - 1 + pktsize) / pktsize;
      xfersize = pktcnt * pktsize;
    }

  oeps[ep_num].xfersize = xfersize;

  USB_DOUTEPS[ep_num].TSIZ = (USB_DOUTEPS[ep_num].TSIZ & ~0x1fffffff)
                             | (xfersize << 0)
                             | (pktcnt << 19);
  USB_DOUTEPS[ep_num].DMAADDR = (uint32_t) buf;
  USB_DOUTEPS[ep_num].CTL = (USB_DOUTEPS[ep_num].CTL & ~DEPCTL_WO_MASK)
                            | USB_DOEP_CTL_CNAK
                            | USB_DOEP_CTL_EPENA;
}

void
usb_lld_setup_endp (struct usb_dev *dev, int ep_num, int ep_type,
                    int rx_en, int tx_en, int maxpacket)
{
  uint32_t ctl;
  uint32_t mps;

  mps = maxpacket;

  if (ep_num == 0)
    {
      if (maxpacket == 64)
        mps = 0;
      else if (maxpacket == 32)
        mps = 1;
      else if (maxpacket == 16)
        mps = 2;
      else if (maxpacket == 8)
        mps = 3;
      else
        mps = 0;
    }

  if (tx_en)
  {
    /* enpoint is not active */

    ctl = (mps << 0)                    /* max packet size */
          | (ep_num << 22)              /* TX FIFO number */
          | USB_DIEP_CTL_SETD0PIDEF
          | USB_DIEP_CTL_USBACTEP
          | USB_DIEP_CTL_SNAK;

    switch (ep_type)
      {
        case EP_CONTROL:
          ctl |= USB_DIEP_CTL_EPTYPE_CONTROL;
          break;
        case EP_BULK:
          ctl |= USB_DIEP_CTL_EPTYPE_BULK;
          ctl &= ~USB_DIEP_CTL_STALL;
          break;
        case EP_ISOCHRONOUS:
          ctl |= USB_DIEP_CTL_EPTYPE_ISO;
          break;
        case EP_INTERRUPT:
          ctl |= USB_DIEP_CTL_EPTYPE_INT;
          ctl &= ~USB_DIEP_CTL_STALL;
          break;
      }
    USB_DINEPS[ep_num].CTL = ctl;

    /* enable interrupt for this endpoint */
    USB->DAINTMSK |= (1 << ep_num);
  }

  if (rx_en)
  {
    ctl = (mps << 0)                     /* max packet size */
          | USB_DOEP_CTL_SETD0PIDEF
          | USB_DOEP_CTL_USBACTEP
          | USB_DOEP_CTL_SNAK;

    switch (ep_type)
      {
        case EP_CONTROL:
          ctl |= USB_DOEP_CTL_EPTYPE_CONTROL;
          break;
        case EP_BULK:
          ctl |= USB_DOEP_CTL_EPTYPE_BULK;
          ctl &= ~USB_DOEP_CTL_STALL;
          break;
        case EP_ISOCHRONOUS:
          ctl |= USB_DOEP_CTL_EPTYPE_ISO;
          break;
        case EP_INTERRUPT:
          ctl |= USB_DOEP_CTL_EPTYPE_INT;
          ctl &= ~USB_DOEP_CTL_STALL;
          break;
      }
    USB_DOUTEPS[ep_num].CTL = ctl;

    /* enable interrupt for this endpoint */
    USB->DAINTMSK |= (1 << (ep_num + 16));
  }

  if (ep_num == 0)
    {
      USB->DCTL = (USB->DCTL & ~DCTL_WO_MASK) | USB_DCTL_CGNPINNAK;
      efm32hg_prepare_ep0_setup (dev);
    }
}

void
usb_lld_stall_rx (int ep_num)
{
  efm32hg_ep_out_stall (ep_num);
}

void
usb_lld_stall_tx (int ep_num)
{
  efm32hg_ep_in_stall (ep_num);
}
