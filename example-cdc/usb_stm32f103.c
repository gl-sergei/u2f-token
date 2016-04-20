#include <stdint.h>
#include <stdlib.h>

#include "sys.h"
#include "usb_lld.h"

#define USB_MAX_PACKET_SIZE 64	/* For FS device */

enum STANDARD_REQUESTS
{
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

/* The state machine states of a control pipe */
enum CONTROL_STATE
{
  WAIT_SETUP,
  SETTING_UP,
  IN_DATA,
  OUT_DATA,
  LAST_IN_DATA,
  WAIT_STATUS_IN,
  WAIT_STATUS_OUT,
  STALLED,
  PAUSE
};

enum FEATURE_SELECTOR
{
  ENDPOINT_STALL,
  DEVICE_REMOTE_WAKEUP
};

struct DATA_INFO
{
  uint8_t *addr;
  uint16_t len;
  uint8_t require_zlp;
};


struct DEVICE_INFO
{
  uint8_t current_configuration;
  uint8_t current_feature;
  uint8_t state;
  /**/
  uint8_t bmRequestType;
  uint8_t bRequest;
  /**/
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

static struct DEVICE_INFO device_info;
static struct DATA_INFO data_info;

static struct DEVICE_INFO *const dev_p = &device_info;
static struct DATA_INFO *const data_p = &data_info;

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

static void usb_handle_transfer (uint16_t istr_value);

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

void usb_lld_init (uint8_t feature)
{
  usb_lld_sys_init ();

  dev_p->state = IN_DATA;

  usb_lld_set_configuration (0);
  dev_p->current_feature = feature;

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

void
usb_interrupt_handler (void)
{
  uint16_t istr_value = st103_get_istr ();

  if ((istr_value & ISTR_RESET))
    {
      st103_set_istr (CLR_RESET);
      usb_cb_device_reset ();
    }
  else
    {
      if ((istr_value & ISTR_OVR))
	st103_set_istr (CLR_OVR);

      if ((istr_value & ISTR_ERR))
	st103_set_istr (CLR_ERR);

      if ((istr_value & ISTR_CTR))
	usb_handle_transfer (istr_value);
    }
}

static void handle_datastage_out (void)
{
  if (data_p->addr && data_p->len)
    {
      uint32_t len = st103_get_rx_count (ENDP0);

      if (len > data_p->len)
	len = data_p->len;

      usb_lld_from_pmabuf (data_p->addr, st103_get_rx_addr (ENDP0), len);
      data_p->len -= len;
      data_p->addr += len;
    }

  if (data_p->len == 0)
    {
      dev_p->state = WAIT_STATUS_IN;
      st103_set_tx_count (ENDP0, 0);
      st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_VALID);
    }
  else
    {
      dev_p->state = OUT_DATA;
      st103_ep_set_rx_status (ENDP0, EP_RX_VALID);
    }
}

static void handle_datastage_in (void)
{
  uint32_t len = USB_MAX_PACKET_SIZE;;

  if ((data_p->len == 0) && (dev_p->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  st103_set_tx_count (ENDP0, 0);
	  st103_ep_set_rxtx_status (ENDP0, EP_RX_VALID, EP_TX_VALID);
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.*/
	  dev_p->state = WAIT_STATUS_OUT;
	  st103_ep_set_rxtx_status (ENDP0, EP_RX_VALID, EP_TX_STALL);
	}

      return;
    }

  dev_p->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  usb_lld_to_pmabuf (data_p->addr, st103_get_tx_addr (ENDP0), len);
  data_p->len -= len;
  data_p->addr += len;
  st103_set_tx_count (ENDP0, len);
  st103_ep_set_tx_status (ENDP0, EP_TX_VALID);
}

typedef int (*HANDLER) (uint8_t req, struct req_args *arg);

static int std_none (uint8_t req, struct req_args *arg)
{
  (void)req; (void)arg;
  return USB_UNSUPPORT;
}

static int std_get_status (uint8_t req, struct req_args *arg)
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
	  uint8_t feature = dev_p->current_feature;

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

      if (dev_p->current_configuration == 0)
	return USB_UNSUPPORT;

      r = usb_cb_interface (USB_QUERY_INTERFACE, arg);
      if (r != USB_SUCCESS)
	return USB_UNSUPPORT;

      return usb_lld_reply_request (&status_info, 2, arg);
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t endpoint = (arg->index & 0x0f);
      uint16_t status;

      if ((arg->index & 0x70) || endpoint == ENDP0)
	return USB_UNSUPPORT;

      if ((arg->index & 0x80))
	{
	  status = st103_ep_get_tx_status (endpoint);
	  if (status == 0)		/* Disabled */
	    return USB_UNSUPPORT;
	  else if (status == EP_TX_STALL)
	    status_info |= 1; /* IN Endpoint stalled */
	}
      else
	{
	  status = st103_ep_get_rx_status (endpoint);
	  if (status == 0)		/* Disabled */
	    return USB_UNSUPPORT;
	  else if (status == EP_RX_STALL)
	    status_info |= 1; /* OUT Endpoint stalled */
	}

      return usb_lld_reply_request (&status_info, 2, arg);
    }

  return USB_UNSUPPORT;
}

static int std_clear_feature (uint8_t req, struct req_args *arg)
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
	  dev_p->current_feature &= ~(1 << 5);
	  return USB_SUCCESS;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t endpoint = (arg->index & 0x0f);
      uint16_t status;

      if (dev_p->current_configuration == 0)
	return USB_UNSUPPORT;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != ENDPOINT_STALL || endpoint == ENDP0)
	return USB_UNSUPPORT;

      if ((arg->index & 0x80))
	status = st103_ep_get_tx_status (endpoint);
      else
	status = st103_ep_get_rx_status (endpoint);

      if (status == 0)		/* Disabled */
	return USB_UNSUPPORT;

      if (arg->index & 0x80)		/* IN endpoint */
	st103_ep_clear_dtog_tx (endpoint);
      else			/* OUT endpoint */
	st103_ep_clear_dtog_rx (endpoint);

      // event??
      return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

static int std_set_feature (uint8_t req, struct req_args *arg)
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
	  dev_p->current_feature |= 1 << 5;
	  // event??
	  return USB_SUCCESS;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t endpoint = (arg->index & 0x0f);
      uint32_t status;

      if (dev_p->current_configuration == 0)
	return USB_UNSUPPORT;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != 0 || endpoint == ENDP0)
	return USB_UNSUPPORT;

      if ((arg->index & 0x80))
	status = st103_ep_get_tx_status (endpoint);
      else
	status = st103_ep_get_rx_status (endpoint);

      if (status == 0)		/* Disabled */
	return USB_UNSUPPORT;

      if (arg->index & 0x80)
	/* IN endpoint */
	st103_ep_set_tx_status (endpoint, EP_TX_STALL);
      else
	/* OUT endpoint */
	st103_ep_set_rx_status (endpoint, EP_RX_STALL);

      // event??
      return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

static int std_set_address (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT && arg->len == 0 && arg->value <= 127
      && arg->index == 0 && dev_p->current_configuration == 0)
    return USB_SUCCESS;

  return USB_UNSUPPORT;
}

static int std_get_descriptor (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  return usb_cb_get_descriptor (rcp, (arg->value >> 8),
				(arg->value & 0xff), arg);
}

static int std_get_configuration (uint8_t req,  struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT)
    return usb_lld_reply_request (&dev_p->current_configuration, 1, arg);

  return USB_UNSUPPORT;
}

static int std_set_configuration (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req))
    return USB_UNSUPPORT;

  if (rcp == DEVICE_RECIPIENT && arg->index == 0 && arg->len == 0)
    return usb_cb_handle_event (USB_EVENT_CONFIG, arg->value);

  return USB_UNSUPPORT;
}

static int std_get_interface (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_SET (req))
    return USB_UNSUPPORT;

  if (rcp == INTERFACE_RECIPIENT)
    {
      if (arg->value != 0 || (arg->index >> 8) != 0 || arg->len != 1)
	return USB_UNSUPPORT;

      if (dev_p->current_configuration == 0)
	return USB_UNSUPPORT;

      return usb_cb_interface (USB_GET_INTERFACE, arg);
    }

  return USB_UNSUPPORT;
}

static int std_set_interface (uint8_t req, struct req_args *arg)
{
  uint8_t rcp = req & RECIPIENT;

  if (USB_SETUP_GET (req) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev_p->current_configuration == 0)
    return USB_UNSUPPORT;

  return usb_cb_interface (USB_SET_INTERFACE, arg);
}


static void handle_setup0 (void)
{
  const uint16_t *pw;
  uint16_t w;
  uint8_t req_no;
  int r = USB_UNSUPPORT;
  HANDLER handler;

  pw = (uint16_t *)(PMA_ADDR + (uint8_t *)(st103_get_rx_addr (ENDP0) * 2));
  w = *pw++;

  dev_p->bmRequestType = w & 0xff;
  dev_p->bRequest = req_no = w >> 8;
  pw++;
  dev_p->value = *pw++;
  pw++;
  dev_p->index = *pw++;
  pw++;
  dev_p->len = *pw;

  data_p->addr = NULL;
  data_p->len = 0;
  data_p->require_zlp = 0;

  if ((dev_p->bmRequestType & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      if (req_no < TOTAL_REQUEST)
	{
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

	  r = (*handler) (dev_p->bmRequestType,
			  (struct req_args *)&dev_p->value);
	}
    }
  else
    r = usb_cb_setup (dev_p->bmRequestType, req_no,
		      (struct req_args *)&dev_p->value);

  if (r != USB_SUCCESS)
    dev_p->state = STALLED;
  else
    {
      if (USB_SETUP_SET (dev_p->bmRequestType))
	{
	  if (dev_p->len == 0)
	    {
	      dev_p->state = WAIT_STATUS_IN;
	      st103_set_tx_count (ENDP0, 0);
	      st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_VALID);
	    }
	  else
	    {
	      dev_p->state = OUT_DATA;
	      st103_ep_set_rx_status (ENDP0, EP_RX_VALID);
	    }
	}
    }
}

static void handle_in0 (void)
{
  if (dev_p->state == IN_DATA || dev_p->state == LAST_IN_DATA)
    handle_datastage_in ();
  else if (dev_p->state == WAIT_STATUS_IN)
    {
      if ((dev_p->bRequest == SET_ADDRESS) &&
	  ((dev_p->bmRequestType & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	{
	  st103_set_daddr (dev_p->value);
	  usb_cb_handle_event (USB_EVENT_ADDRESS, dev_p->value);
	}
      else
	usb_cb_ctrl_write_finish (dev_p->bmRequestType, dev_p->bRequest,
				  (struct req_args *)&dev_p->value);

      dev_p->state = STALLED;
    }
  else
    dev_p->state = STALLED;
}

static void handle_out0 (void)
{
  if (dev_p->state == IN_DATA || dev_p->state == LAST_IN_DATA)
    /* host aborts the transfer before finish */
    dev_p->state = STALLED;
  else if (dev_p->state == OUT_DATA)
    handle_datastage_out ();
  else if (dev_p->state == WAIT_STATUS_OUT)
    dev_p->state = STALLED;
  /* Unexpect state, STALL the endpoint */
  else
    dev_p->state = STALLED;
}

static void
usb_handle_transfer (uint16_t istr_value)
{
  uint16_t ep_value = 0;
  uint8_t ep_index;

  ep_index = (istr_value & ISTR_EP_ID);
  /* Decode and service non control endpoints interrupt  */
  /* process related endpoint register */
  ep_value = st103_get_epreg (ep_index);

  if (ep_index == 0)
    {
      if ((ep_value & EP_CTR_TX))
	{
	  st103_ep_clear_ctr_tx (ep_index);
	  handle_in0 ();
	}

      if ((ep_value & EP_CTR_RX))
	{
	  st103_ep_clear_ctr_rx (ep_index);

	  if ((ep_value & EP_SETUP))
	    handle_setup0 ();
	  else
	    handle_out0 ();
	}

      if (dev_p->state == STALLED)
	st103_ep_set_rxtx_status (ENDP0, EP_RX_STALL, EP_TX_STALL);
    }
  else
    {
      if ((ep_value & EP_CTR_RX))
	{
	  st103_ep_clear_ctr_rx (ep_index);
	  usb_cb_rx_ready (ep_index);
	}

      if ((ep_value & EP_CTR_TX))
	{
	  st103_ep_clear_ctr_tx (ep_index);
	  usb_cb_tx_done (ep_index);
	}
    }
}

void usb_lld_reset (uint8_t feature)
{
  usb_lld_set_configuration (0);
  dev_p->current_feature = feature;
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

int usb_lld_tx_data_len (int ep_num)
{
  return st103_get_tx_count (ep_num);
}

int usb_lld_rx_data_len (int ep_num)
{
  return st103_get_rx_count (ep_num);
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
      ep_rxtx_status |= EP_RX_VALID;
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

void usb_lld_set_configuration (uint8_t config)
{
  dev_p->current_configuration = config;
}

uint8_t usb_lld_current_configuration (void)
{
  return dev_p->current_configuration;
}

void usb_lld_set_data_to_recv (void *p, size_t len)
{
  data_p->addr = p;
  data_p->len = len;
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
usb_lld_reply_request (const void *buf, size_t buflen, struct req_args *ctl)
{
  uint32_t len_asked = ctl->len;
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
      dev_p->state = LAST_IN_DATA;
    }
  else
    {
      len = USB_MAX_PACKET_SIZE;
      dev_p->state = IN_DATA;
    }

  if (len)
    {
      usb_lld_to_pmabuf (data_p->addr, st103_get_tx_addr (ENDP0), len);
      data_p->len -= len;
      data_p->addr += len;
    }

  st103_set_tx_count (ENDP0, len);
  st103_ep_set_tx_status (ENDP0, EP_TX_VALID);
  return USB_SUCCESS;
}
