/*
 * usb-stm32f103.c - USB driver for STM32F103
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

#include "sys-stm32f103.h"
#include "usb_lld.h"
#include "usb_lld_driver.h"

#define REG_BASE  (0x40005C00UL) /* USB_IP Peripheral Registers base address */
#define PMA_ADDR  (0x40006000UL) /* USB_IP Packet Memory Area base address   */

/* Control register */
#define CNTR    ((volatile uint16_t *)(REG_BASE + 0x40))
/* Interrupt status register */
#define ISTR    ((volatile uint16_t *)(REG_BASE + 0x44))
/* Frame number register */
#define FNR     ((volatile uint16_t *)(REG_BASE + 0x48))
/* Device address register */
#define DADDR   ((volatile uint16_t *)(REG_BASE + 0x4C))
/* Buffer Table address register */
#define BTABLE  ((volatile uint16_t *)(REG_BASE + 0x50))

#define ISTR_CTR    (0x8000) /* Correct TRansfer (read-only bit) */
#define ISTR_OVR    (0x4000) /* OVeR/underrun (clear-only bit) */
#define ISTR_ERR    (0x2000) /* ERRor (clear-only bit) */
#define ISTR_WKUP   (0x1000) /* WaKe UP (clear-only bit) */
#define ISTR_SUSP   (0x0800) /* SUSPend (clear-only bit) */
#define ISTR_RESET  (0x0400) /* RESET (clear-only bit) */
#define ISTR_SOF    (0x0200) /* Start Of Frame (clear-only bit) */
#define ISTR_ESOF   (0x0100) /* Expected Start Of Frame (clear-only bit) */

#define ISTR_DIR    (0x0010)  /* DIRection of transaction (read-only bit)  */
#define ISTR_EP_ID  (0x000F)  /* EndPoint IDentifier (read-only bit)  */

#define CLR_OVR    (~ISTR_OVR)   /* clear OVeR/underrun bit*/
#define CLR_ERR    (~ISTR_ERR)   /* clear ERRor bit */
#define CLR_WKUP   (~ISTR_WKUP)  /* clear WaKe UP bit     */
#define CLR_SUSP   (~ISTR_SUSP)  /* clear SUSPend bit     */
#define CLR_RESET  (~ISTR_RESET) /* clear RESET bit      */
#define CLR_SOF    (~ISTR_SOF)   /* clear Start Of Frame bit   */
#define CLR_ESOF   (~ISTR_ESOF)  /* clear Expected Start Of Frame bit */

#define CNTR_CTRM   (0x8000) /* Correct TRansfer Mask */
#define CNTR_OVRM   (0x4000) /* OVeR/underrun Mask */
#define CNTR_ERRM   (0x2000) /* ERRor Mask */
#define CNTR_WKUPM  (0x1000) /* WaKe UP Mask */
#define CNTR_SUSPM  (0x0800) /* SUSPend Mask */
#define CNTR_RESETM (0x0400) /* RESET Mask   */
#define CNTR_SOFM   (0x0200) /* Start Of Frame Mask */
#define CNTR_ESOFM  (0x0100) /* Expected Start Of Frame Mask */

#define CNTR_RESUME (0x0010) /* RESUME request */
#define CNTR_FSUSP  (0x0008) /* Force SUSPend */
#define CNTR_LPMODE (0x0004) /* Low-power MODE */
#define CNTR_PDWN   (0x0002) /* Power DoWN */
#define CNTR_FRES   (0x0001) /* Force USB RESet */

#define DADDR_EF (0x80)
#define DADDR_ADD (0x7F)

#define EP_CTR_RX      (0x8000) /* EndPoint Correct TRansfer RX */
#define EP_DTOG_RX     (0x4000) /* EndPoint Data TOGGLE RX */
#define EPRX_STAT      (0x3000) /* EndPoint RX STATus bit field */
#define EP_SETUP       (0x0800) /* EndPoint SETUP */
#define EP_T_FIELD     (0x0600) /* EndPoint TYPE */
#define EP_KIND        (0x0100) /* EndPoint KIND */
#define EP_CTR_TX      (0x0080) /* EndPoint Correct TRansfer TX */
#define EP_DTOG_TX     (0x0040) /* EndPoint Data TOGGLE TX */
#define EPTX_STAT      (0x0030) /* EndPoint TX STATus bit field */
#define EPADDR_FIELD   (0x000F) /* EndPoint ADDRess FIELD */

#define EPREG_MASK     (EP_CTR_RX|EP_SETUP|EP_T_FIELD|EP_KIND|EP_CTR_TX|EPADDR_FIELD)

/* STAT_TX[1:0] STATus for TX transfer */
#define EP_TX_DIS      (0x0000) /* EndPoint TX DISabled */
#define EP_TX_STALL    (0x0010) /* EndPoint TX STALLed */
#define EP_TX_NAK      (0x0020) /* EndPoint TX NAKed */
#define EP_TX_VALID    (0x0030) /* EndPoint TX VALID */
#define EPTX_DTOG1     (0x0010) /* EndPoint TX Data TOGgle bit1 */
#define EPTX_DTOG2     (0x0020) /* EndPoint TX Data TOGgle bit2 */

/* STAT_RX[1:0] STATus for RX transfer */
#define EP_RX_DIS      (0x0000) /* EndPoint RX DISabled */
#define EP_RX_STALL    (0x1000) /* EndPoint RX STALLed */
#define EP_RX_NAK      (0x2000) /* EndPoint RX NAKed */
#define EP_RX_VALID    (0x3000) /* EndPoint RX VALID */
#define EPRX_DTOG1     (0x1000) /* EndPoint RX Data TOGgle bit1 */
#define EPRX_DTOG2     (0x2000) /* EndPoint RX Data TOGgle bit1 */

static int usb_handle_transfer (struct usb_dev *dev, uint16_t istr_value);

static void st103_set_btable (void)
{
  *BTABLE = 0;
}

static uint16_t st103_get_istr (void)
{
  return *ISTR;
}

static void st103_set_istr (uint16_t istr)
{
  *ISTR = istr;
}

static void st103_set_cntr (uint16_t cntr)
{
  *CNTR = cntr;
}

static void st103_set_daddr (uint16_t daddr)
{
  *DADDR  = daddr | DADDR_EF;
}

static void st103_set_epreg (uint8_t ep_num, uint16_t value)
{
  uint16_t *reg_p = (uint16_t *)(REG_BASE + ep_num*4);

  *reg_p = value;
}

static uint16_t st103_get_epreg (uint8_t ep_num)
{
  uint16_t *reg_p = (uint16_t *)(REG_BASE + ep_num*4);

  return *reg_p;
}

static void st103_set_tx_addr (uint8_t ep_num, uint16_t addr)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+0)*2);

  *reg_p = addr;
}

static uint16_t st103_get_tx_addr (uint8_t ep_num)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+0)*2);

  return *reg_p;
}


static void st103_set_tx_count (uint8_t ep_num, uint16_t size)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+2)*2);

  *reg_p = size;
}

static uint16_t st103_get_tx_count (uint8_t ep_num)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+2)*2);

  return *reg_p  & 0x03ff;
}


static void st103_set_rx_addr (uint8_t ep_num, uint16_t addr)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+4)*2);

  *reg_p = addr;
}

static uint16_t st103_get_rx_addr (uint8_t ep_num)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+4)*2);

  return *reg_p;
}


static void st103_set_rx_buf_size (uint8_t ep_num, uint16_t size)
{				/* Assume size is even */
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+6)*2);
  uint16_t value;

  if (size <= 62)
    value = (size & 0x3e) << 9;
  else
    value = 0x8000 | (((size >> 5) - 1) << 10);

  *reg_p = value;
}

static uint16_t st103_get_rx_count (uint8_t ep_num)
{
  uint16_t *reg_p = (uint16_t *)(PMA_ADDR + (ep_num*8+6)*2);

  return *reg_p & 0x03ff;
}


static void st103_ep_clear_ctr_rx (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num) & ~EP_CTR_RX & EPREG_MASK;

  st103_set_epreg (ep_num, value);
}

static void st103_ep_clear_ctr_tx (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num) & ~EP_CTR_TX & EPREG_MASK;

  st103_set_epreg (ep_num, value);
}

static void st103_ep_set_rxtx_status (uint8_t ep_num, uint16_t st_rx,
				      uint16_t st_tx)
{
  uint16_t value = st103_get_epreg (ep_num);

  value &= (EPREG_MASK|EPRX_STAT|EPTX_STAT);
  value ^= (EPRX_DTOG1 & st_rx);
  value ^= (EPRX_DTOG2 & st_rx);
  value ^= (EPTX_DTOG1 & st_tx);
  value ^= (EPTX_DTOG2 & st_tx);
  value |= EP_CTR_RX | EP_CTR_TX;
  st103_set_epreg (ep_num, value);
}

static void st103_ep_set_rx_status (uint8_t ep_num, uint16_t st_rx)
{
  uint16_t value = st103_get_epreg (ep_num);

  value &= (EPREG_MASK|EPRX_STAT);
  value ^= (EPRX_DTOG1 & st_rx);
  value ^= (EPRX_DTOG2 & st_rx);
  value |= EP_CTR_RX | EP_CTR_TX;
  st103_set_epreg (ep_num, value);
}

static uint16_t st103_ep_get_rx_status (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num);

  return value & EPRX_STAT;
}

static void st103_ep_set_tx_status (uint8_t ep_num, uint16_t st_tx)
{
  uint16_t value = st103_get_epreg (ep_num);

  value &= (EPREG_MASK|EPTX_STAT);
  value ^= (EPTX_DTOG1 & st_tx);
  value ^= (EPTX_DTOG2 & st_tx);
  value |= EP_CTR_RX | EP_CTR_TX;
  st103_set_epreg (ep_num, value);
}

static uint16_t st103_ep_get_tx_status (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num);

  return value & EPTX_STAT;
}

static void st103_ep_clear_dtog_rx (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num);

  if ((value & EP_DTOG_RX))
    {
      value &= EPREG_MASK;
      value |= EP_CTR_RX | EP_CTR_TX | EP_DTOG_RX;
      st103_set_epreg (ep_num, value);
    }
}

static void st103_ep_clear_dtog_tx (uint8_t ep_num)
{
  uint16_t value = st103_get_epreg (ep_num);

  if ((value & EP_DTOG_TX))
    {
      value &= EPREG_MASK;
      value |= EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX;
      st103_set_epreg (ep_num, value);
    }
}

void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  dev->state = STALLED;
  st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_STALL);
}

int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  dev->state = WAIT_STATUS_IN;
  st103_set_tx_count (ENDP0, 0);
  st103_ep_set_rxtx_status (ENDP0, EP_RX_NAK, EP_TX_VALID);
  return USB_EVENT_OK;
}

void usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_sys_init ();

  dev->configuration = 0;
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  /* Reset USB */
  st103_set_cntr (CNTR_FRES);
  st103_set_cntr (0);

  /* Clear Interrupt Status Register, and enable interrupt for USB */
  st103_set_istr (0);
  st103_set_cntr (CNTR_CTRM | CNTR_RESETM);
}

void usb_lld_prepare_shutdown (void)
{
  st103_set_istr (0);
  st103_set_cntr (0);
}

void usb_lld_shutdown (void)
{
  st103_set_cntr (CNTR_PDWN);
  usb_lld_sys_shutdown ();
}

#define USB_MAKE_EV(event) (event<<24)
#define USB_MAKE_TXRX(ep_num,txrx,len) ((txrx? (1<<23):0)|(ep_num<<16)|len)

int
usb_lld_event_handler (struct usb_dev *dev)
{
  uint16_t istr_value = st103_get_istr ();

  if ((istr_value & ISTR_RESET))
    {
      st103_set_istr (CLR_RESET);
      return USB_MAKE_EV (USB_EVENT_DEVICE_RESET);
    }
  else
    {
      if ((istr_value & ISTR_OVR))
	st103_set_istr (CLR_OVR);

      if ((istr_value & ISTR_ERR))
	st103_set_istr (CLR_ERR);

      if ((istr_value & ISTR_CTR))
	return usb_handle_transfer (dev, istr_value);
    }

  return USB_EVENT_OK;
}

static void handle_datastage_out (struct usb_dev *dev)
{
  if (dev->ctrl_data.addr && dev->ctrl_data.len)
    {
      uint32_t len = st103_get_rx_count (ENDP0);

      if (len > dev->ctrl_data.len)
	len = dev->ctrl_data.len;

      usb_lld_from_pmabuf (dev->ctrl_data.addr, st103_get_rx_addr (ENDP0), len);
      dev->ctrl_data.len -= len;
      dev->ctrl_data.addr += len;
    }

  if (dev->ctrl_data.len == 0)
    {
      dev->state = WAIT_STATUS_IN;
      st103_set_tx_count (ENDP0, 0);
      st103_ep_set_tx_status (ENDP0, EP_TX_VALID);
    }
  else
    {
      dev->state = OUT_DATA;
      st103_ep_set_rx_status (ENDP0, EP_RX_VALID);
    }
}

static void handle_datastage_in (struct usb_dev *dev)
{
  uint32_t len = USB_MAX_PACKET_SIZE;;
  struct ctrl_data *data_p = &dev->ctrl_data;

  if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  st103_set_tx_count (ENDP0, 0);
	  st103_ep_set_tx_status (ENDP0, EP_TX_VALID);
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.  */
	  dev->state = WAIT_STATUS_OUT;
	  st103_ep_set_rx_status (ENDP0, EP_RX_VALID);
	}

      return;
    }

  dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  usb_lld_to_pmabuf (data_p->addr, st103_get_tx_addr (ENDP0), len);
  data_p->len -= len;
  data_p->addr += len;
  st103_set_tx_count (ENDP0, len);
  st103_ep_set_tx_status (ENDP0, EP_TX_VALID);
}

typedef int (*HANDLER) (struct usb_dev *dev);

static int std_none (struct usb_dev *dev)
{
  (void)dev;
  return -1;
}

static int std_get_status (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = (arg->type & RECIPIENT);
  uint16_t status_info = 0;

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
      uint8_t endpoint = (arg->index & 0x0f);
      uint16_t status;

      if ((arg->index & 0x70) || endpoint == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	{
	  status = st103_ep_get_tx_status (endpoint);
	  if (status == 0)		/* Disabled */
	    return -1;
	  else if (status == EP_TX_STALL)
	    status_info |= 1; /* IN Endpoint stalled */
	}
      else
	{
	  status = st103_ep_get_rx_status (endpoint);
	  if (status == 0)		/* Disabled */
	    return -1;
	  else if (status == EP_RX_STALL)
	    status_info |= 1; /* OUT Endpoint stalled */
	}

      return usb_lld_ctrl_send (dev, &status_info, 2);
    }

  return -1;
}

static int std_clear_feature (struct usb_dev *dev)
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
      uint8_t endpoint = (arg->index & 0x0f);
      uint16_t status;

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || endpoint == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	status = st103_ep_get_tx_status (endpoint);
      else
	status = st103_ep_get_rx_status (endpoint);

      if (status == 0)		/* It's disabled endpoint.  */
	return -1;

      if (arg->index & 0x80)	/* IN endpoint */
	st103_ep_clear_dtog_tx (endpoint);
      else			/* OUT endpoint */
	st103_ep_clear_dtog_rx (endpoint);

      return USB_EVENT_CLEAR_FEATURE_ENDPOINT;
    }

  return -1;
}

static int std_set_feature (struct usb_dev *dev)
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
      uint8_t endpoint = (arg->index & 0x0f);
      uint32_t status;

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || endpoint == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	status = st103_ep_get_tx_status (endpoint);
      else
	status = st103_ep_get_rx_status (endpoint);

      if (status == 0)		/* It's disabled endpoint.  */
	return -1;

      if (arg->index & 0x80)	/* IN endpoint */
	st103_ep_set_tx_status (endpoint, EP_TX_STALL);
      else			/* OUT endpoint */
	st103_ep_set_rx_status (endpoint, EP_RX_STALL);

      return USB_EVENT_SET_FEATURE_ENDPOINT;
    }

  return -1;
}

static int std_set_address (struct usb_dev *dev)
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

static int std_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  if (USB_SETUP_SET (arg->type))
    return -1;

  return USB_EVENT_GET_DESCRIPTOR;
}

static int std_get_configuration (struct usb_dev *dev)
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

static int std_set_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (arg->index != 0 || arg->len != 0)
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    return USB_EVENT_SET_CONFIGURATION;

  return -1;
}

static int std_get_interface (struct usb_dev *dev)
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

static int std_set_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev->configuration == 0)
    return -1;

  return USB_EVENT_SET_INTERFACE;
}


static int handle_setup0 (struct usb_dev *dev)
{
  const uint16_t *pw;
  uint16_t w;
  uint8_t req_no;
  HANDLER handler;

  pw = (uint16_t *)(PMA_ADDR + (uint8_t *)(st103_get_rx_addr (ENDP0) * 2));
  w = *pw++;

  dev->dev_req.type = (w & 0xff);
  dev->dev_req.request = req_no = (w >> 8);
  pw++;
  dev->dev_req.value = *pw++;
  pw++;
  dev->dev_req.index = *pw++;
  pw++;
  dev->dev_req.len = *pw;

  dev->ctrl_data.addr = NULL;
  dev->ctrl_data.len = 0;
  dev->ctrl_data.require_zlp = 0;

  if ((dev->dev_req.type & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      int r;

      switch (req_no)
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

static int handle_in0 (struct usb_dev *dev)
{
  int r = 0;

  if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    handle_datastage_in (dev);
  else if (dev->state == WAIT_STATUS_IN)
    {
      dev->state = WAIT_SETUP;

      if ((dev->dev_req.request == SET_ADDRESS) &&
	  ((dev->dev_req.type & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	{
	  st103_set_daddr (dev->dev_req.value);
	  r = USB_EVENT_DEVICE_ADDRESSED;
	}
      else
	r = USB_EVENT_CTRL_WRITE_FINISH;
    }
  else
    {
      dev->state = STALLED;
      st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_STALL);
    }

  return r;
}

static void handle_out0 (struct usb_dev *dev)
{
  if (dev->state == OUT_DATA)
    /* Usual case.  */
    handle_datastage_out (dev);
  else if (dev->state == WAIT_STATUS_OUT)
    /*
     * Control READ transfer finished by ZLP.
     * Leave ENDP0 status RX_NAK, TX_NAK.
     */
    dev->state = WAIT_SETUP;
  else
    {
      /*
       * dev->state == IN_DATA || dev->state == LAST_IN_DATA
       * (Host aborts the transfer before finish)
       * Or else, unexpected state.
       * STALL the endpoint, until we receive the next SETUP token.
       */
      dev->state = STALLED;
      st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_STALL);
    }
}


static int
usb_handle_transfer (struct usb_dev *dev, uint16_t istr_value)
{
  uint16_t ep_value = 0;
  uint8_t ep_num = (istr_value & ISTR_EP_ID);

  ep_value = st103_get_epreg (ep_num);

  if (ep_num == 0)
    {
      if ((ep_value & EP_CTR_TX))
	{
	  st103_ep_clear_ctr_tx (ep_num);
	  return USB_MAKE_EV (handle_in0 (dev));
	}

      if ((ep_value & EP_CTR_RX))
	{
	  st103_ep_clear_ctr_rx (ep_num);

	  if ((ep_value & EP_SETUP))
	    return USB_MAKE_EV (handle_setup0 (dev));
	  else
	    {
	      handle_out0 (dev);
	      return USB_EVENT_OK;
	    }
	}
    }
  else
    {
      uint16_t len;

      if ((ep_value & EP_CTR_RX))
	{
	  len = st103_get_rx_count (ep_num);
	  st103_ep_clear_ctr_rx (ep_num);
	  return USB_MAKE_TXRX (ep_num, 0, len);
	}

      if ((ep_value & EP_CTR_TX))
	{
	  len = st103_get_tx_count (ep_num);
	  st103_ep_clear_ctr_tx (ep_num);
	  return  USB_MAKE_TXRX (ep_num, 1, len);
	}
    }

  return USB_EVENT_OK;
}

void usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  st103_set_btable ();
  st103_set_daddr (0);
}

void usb_lld_txcpy (const void *src,
		    int ep_num, int offset, size_t len)
{
  usb_lld_to_pmabuf (src, st103_get_tx_addr (ep_num) + offset, len);
}

void usb_lld_write (uint8_t ep_num, const void *buf, size_t len)
{
  usb_lld_to_pmabuf (buf, st103_get_tx_addr (ep_num), len);
  st103_set_tx_count (ep_num, len);
  st103_ep_set_tx_status (ep_num, EP_TX_VALID);
}

void usb_lld_rxcpy (uint8_t *dst,
		    int ep_num, int offset, size_t len)
{
  usb_lld_from_pmabuf (dst, st103_get_rx_addr (ep_num) + offset, len);
}

void usb_lld_tx_enable (int ep_num, size_t len)
{
  st103_set_tx_count (ep_num, len);
  st103_ep_set_tx_status (ep_num, EP_TX_VALID);
}

void usb_lld_stall_tx (int ep_num)
{
  st103_ep_set_tx_status (ep_num, EP_TX_STALL);
}

void usb_lld_stall_rx (int ep_num)
{
  st103_ep_set_rx_status (ep_num, EP_RX_STALL);
}

void usb_lld_rx_enable (int ep_num)
{
  st103_ep_set_rx_status (ep_num, EP_RX_VALID);
}

void usb_lld_setup_endpoint (int ep_num, int ep_type, int ep_kind,
			     int ep_rx_addr, int ep_tx_addr,
			     int ep_rx_buf_size)
{
  uint16_t epreg_value = st103_get_epreg (ep_num);
  uint16_t ep_rxtx_status = 0;	/* Both disabled */

  /* Clear: Write 1 if 1: EP_DTOG_RX, EP_DTOG_TX */
  /* Set: Write:          EP_T_FIELD, EP_KIND, EPADDR_FIELD */
  /* Set: Toggle:         EPRX_STAT, EPTX_STAT */
  epreg_value &= (EPRX_STAT | EP_SETUP | EPTX_STAT | EP_DTOG_RX | EP_DTOG_TX);
#if USB_KEEP_CORRECT_TRANSFER_FLAGS
  /* Keep: Write 1:       EP_CTR_RX, EP_CTR_TX */
  epreg_value |= (EP_CTR_RX|EP_CTR_TX);
#else
  /* Clear: Write 0:      EP_CTR_RX, EP_CTR_TX */
#endif
  epreg_value |= ep_type;
  epreg_value |= ep_kind;
  epreg_value |= ep_num;

  if (ep_rx_addr)
    {
      ep_rxtx_status |= EP_RX_NAK;
      st103_set_rx_addr (ep_num, ep_rx_addr);
      st103_set_rx_buf_size (ep_num, ep_rx_buf_size);
    }

  if (ep_tx_addr)
    {
      ep_rxtx_status |= EP_TX_NAK;
      st103_set_tx_addr (ep_num, ep_tx_addr);
    }

  epreg_value ^= (EPRX_DTOG1 & ep_rxtx_status);
  epreg_value ^= (EPRX_DTOG2 & ep_rxtx_status);
  epreg_value ^= (EPTX_DTOG1 & ep_rxtx_status);
  epreg_value ^= (EPTX_DTOG2 & ep_rxtx_status);

  st103_set_epreg (ep_num, epreg_value);
}

void usb_lld_set_configuration (struct usb_dev *dev, uint8_t config)
{
  dev->configuration = config;
}

uint8_t usb_lld_current_configuration (struct usb_dev *dev)
{
  return dev->configuration;
}

int usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  data_p->addr = p;
  data_p->len = len;
  dev->state = OUT_DATA;
  st103_ep_set_rx_status (ENDP0, EP_RX_VALID);
  return USB_EVENT_OK;
}

void usb_lld_to_pmabuf (const void *src, uint16_t addr, size_t n)
{
  const uint8_t *s = (const uint8_t *)src;
  uint16_t *p;
  uint16_t w;

  if (n == 0)
    return;

  if ((addr & 1))
    {
      p = (uint16_t *)(PMA_ADDR + (addr - 1) * 2);
      w = *p;
      w = (w & 0xff) | (*s++) << 8;
      *p = w;
      p += 2;
      n--;
    }
  else
    p = (uint16_t *)(PMA_ADDR + addr * 2);

  while (n >= 2)
    {
      w = *s++;
      w |= (*s++) << 8;
      *p = w;
      p += 2;
      n -= 2;
    }

  if (n > 0)
    {
      w = *s;
      *p = w;
    }
}

void usb_lld_from_pmabuf (void *dst, uint16_t addr, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  uint16_t *p;
  uint16_t w;

  if (n == 0)
    return;

  if ((addr & 1))
    {
      p = (uint16_t *)(PMA_ADDR + (addr - 1) * 2);
      w = *p;
      *d++ = (w >> 8);
      p += 2;
      n--;
    }
  else
    p = (uint16_t *)(PMA_ADDR + addr * 2);

  while (n >= 2)
    {
      w = *p;
      *d++ = (w & 0xff);
      *d++ = (w >> 8);
      p += 2;
      n -= 2;
    }

  if (n > 0)
    {
      w = *p;
      *d = (w & 0xff);
    }
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

  if (data_p->len != 0 && (data_p->len % USB_MAX_PACKET_SIZE) == 0)
    data_p->require_zlp = 1;

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

  if (len)
    {
      usb_lld_to_pmabuf (data_p->addr, st103_get_tx_addr (ENDP0), len);
      data_p->len -= len;
      data_p->addr += len;
    }

  st103_set_tx_count (ENDP0, len);
  st103_ep_set_rxtx_status (ENDP0, EP_RX_NAK, EP_TX_VALID);
  return USB_EVENT_OK;
}
