#define STANDARD_ENDPOINT_DESC_SIZE             0x09

/* endpoints enumeration */
#define ENDP0       ((uint8_t)0)
#define ENDP1       ((uint8_t)1)
#define ENDP2       ((uint8_t)2)
#define ENDP3       ((uint8_t)3)
#define ENDP4       ((uint8_t)4)
#define ENDP5       ((uint8_t)5)
#define ENDP6       ((uint8_t)6)
#define ENDP7       ((uint8_t)7)

enum RECIPIENT_TYPE
{
  DEVICE_RECIPIENT = 0, /* Recipient device    */
  INTERFACE_RECIPIENT,  /* Recipient interface */
  ENDPOINT_RECIPIENT,   /* Recipient endpoint  */
  OTHER_RECIPIENT
};

enum DESCRIPTOR_TYPE
{
  DEVICE_DESCRIPTOR = 1,
  CONFIG_DESCRIPTOR,
  STRING_DESCRIPTOR,
  INTERFACE_DESCRIPTOR,
  ENDPOINT_DESCRIPTOR
};

#define REQUEST_DIR       0x80  /* Mask to get request dir  */
#define REQUEST_TYPE      0x60  /* Mask to get request type */
#define STANDARD_REQUEST  0x00  /* Standard request         */
#define CLASS_REQUEST     0x20  /* Class request            */
#define VENDOR_REQUEST    0x40  /* Vendor request           */
#define RECIPIENT         0x1F  /* Mask to get recipient    */

#define USB_SETUP_SET(req) ((req & REQUEST_DIR) == 0)
#define USB_SETUP_GET(req) ((req & REQUEST_DIR) != 0)

struct device_req {
  uint8_t type;
  uint8_t request;
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

struct ctrl_data {
  uint8_t *addr;
  uint16_t len;
  uint8_t require_zlp;
};

struct usb_dev {
  uint8_t configuration;
  uint8_t feature;
  uint8_t state;
  struct device_req dev_req;
  struct ctrl_data ctrl_data;
};

enum {
  USB_EVENT_OK=0,		/* Processed in lower layer.  */
  /* Device reset and suspend.  */
  USB_EVENT_DEVICE_RESET,
  USB_EVENT_DEVICE_SUSPEND,
  /* Device Requests (Control WRITE Transfer): Standard */
  USB_EVENT_SET_CONFIGURATION,
  USB_EVENT_SET_INTERFACE,
  USB_EVENT_SET_FEATURE_DEVICE,
  USB_EVENT_SET_FEATURE_ENDPOINT,
  USB_EVENT_CLEAR_FEATURE_DEVICE,
  USB_EVENT_CLEAR_FEATURE_ENDPOINT,
  /* Device Requests (Control READ Transfer): Standard */
  USB_EVENT_GET_STATUS_INTERFACE,
  USB_EVENT_GET_DESCRIPTOR,
  USB_EVENT_GET_INTERFACE,
  /* Device Requests (Control READ/WRITE Transfer): Non-Standard */
  USB_EVENT_CTRL_REQUEST,
  USB_EVENT_CTRL_WRITE_FINISH,
  /* Device addressed.  */
  USB_EVENT_DEVICE_ADDRESSED,
};

enum DEVICE_STATE {
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
};

void usb_lld_init (struct usb_dev *dev, uint8_t feature);

/*
 * Return value is encoded integer:
 *     event-no:    8-bit, 0 if TX/RX
 *     tx/rx-flag:  1-bit, 0 if rx, 1 if tx
 *     endpoint no: 7-bit
 *     length:      16-bit
 */
#define USB_EVENT_TXRX(e) ((e >> 23) & 1)
#define USB_EVENT_LEN(e) (e & 0xffff)
#define USB_EVENT_ENDP(e) ((e >> 16) & 0x7f)
#define USB_EVENT_ID(e) ((e >> 24))
int usb_lld_event_handler (struct usb_dev *dev);

/*
 * Control Endpoint ENDP0 does device requests handling.
 * In response to an event of
 *     USB_EVENT_SET_CONFIGURATION
 *     USB_EVENT_SET_INTERFACE
 *     USB_EVENT_SET_FEATURE_DEVICE
 *     USB_EVENT_SET_FEATURE_ENDPOINT
 *     USB_EVENT_CLEAR_FEATURE_DEVICE
 *     USB_EVENT_CLEAR_FEATURE_ENDPOINT
 *     USB_EVENT_GET_STATUS_INTERFACE
 *     USB_EVENT_GET_DESCRIPTOR
 *     USB_EVENT_GET_INTERFACE
 *     USB_EVENT_CTRL_REQUEST
 *  a single action should be done, which is SEND, RECV, or,
 *  ACKNOWLEDGE (no data to be sent, or to be received).
 *  Otherwise, it's an error.
 */
int usb_lld_ctrl_send (struct usb_dev *dev, const void *buf, size_t buflen);
int usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len);
int usb_lld_ctrl_ack (struct usb_dev *dev);
void usb_lld_ctrl_error (struct usb_dev *dev);

void usb_lld_reset (struct usb_dev *dev, uint8_t feature);
void usb_lld_set_configuration (struct usb_dev *dev, uint8_t config);
uint8_t usb_lld_current_configuration (struct usb_dev *dev);
void usb_lld_prepare_shutdown (void);
void usb_lld_shutdown (void);

#if defined(MCU_KINETIS_L)
void usb_lld_tx_enable_buf (int ep_num, const void *buf, size_t len);
void usb_lld_rx_enable_buf (int ep_num, void *buf, size_t len);

void usb_lld_setup_endp (struct usb_dev *dev, int ep_num, int rx_en, int tx_en);
void usb_lld_stall (int ep_num);
#elif defined(GNU_LINUX_EMULATION)
void usb_lld_tx_enable_buf (int ep_num, const void *buf, size_t len);
void usb_lld_rx_enable_buf (int ep_num, void *buf, size_t len);

void usb_lld_setup_endp (struct usb_dev *dev, int ep_num, int rx_en, int tx_en);
void usb_lld_stall_tx (int ep_num);
void usb_lld_stall_rx (int ep_num);
#else
/* EP_TYPE[1:0] EndPoint TYPE */
#define EP_BULK        (0x0000) /* EndPoint BULK        */
#define EP_CONTROL     (0x0200) /* EndPoint CONTROL     */
#define EP_ISOCHRONOUS (0x0400) /* EndPoint ISOCHRONOUS */
#define EP_INTERRUPT   (0x0600) /* EndPoint INTERRUPT   */

void usb_lld_tx_enable (int ep_num, size_t len);
void usb_lld_rx_enable (int ep_num);

void usb_lld_setup_endpoint (int ep_num, int ep_type, int ep_kind,
			     int ep_rx_addr, int ep_tx_addr,
			     int ep_rx_memory_size);
void usb_lld_stall_tx (int ep_num);
void usb_lld_stall_rx (int ep_num);

void usb_lld_txcpy (const void *src, int ep_num, int offset, size_t len);
void usb_lld_write (uint8_t ep_num, const void *buf, size_t len);
void usb_lld_rxcpy (uint8_t *dst, int ep_num, int offset, size_t len);
void usb_lld_to_pmabuf (const void *src, uint16_t addr, size_t n);
void usb_lld_from_pmabuf (void *dst, uint16_t addr, size_t n);
#endif
