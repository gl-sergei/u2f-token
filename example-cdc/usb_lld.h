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

/* EP_TYPE[1:0] EndPoint TYPE */
#define EP_BULK        (0x0000) /* EndPoint BULK        */
#define EP_CONTROL     (0x0200) /* EndPoint CONTROL     */
#define EP_ISOCHRONOUS (0x0400) /* EndPoint ISOCHRONOUS */
#define EP_INTERRUPT   (0x0600) /* EndPoint INTERRUPT   */

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

enum
{
  USB_UNSUPPORT = 0,
  USB_SUCCESS = 1,
};

struct req_args {
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

void usb_cb_device_reset (void);
int usb_cb_setup (uint8_t req, uint8_t req_no, struct req_args *arg);
int usb_cb_interface (uint8_t cmd, struct req_args *arg);
int usb_cb_get_descriptor (uint8_t rcp, uint8_t desc_type, uint8_t desc_index,
			   struct req_args *arg);
int usb_cb_handle_event (uint8_t event_type, uint16_t value);
void usb_cb_ctrl_write_finish (uint8_t req, uint8_t req_no,
			       struct req_args *arg);
void usb_cb_tx_done (uint8_t ep_num);
void usb_cb_rx_ready (uint8_t ep_num);

enum {
  USB_EVENT_ADDRESS,
  USB_EVENT_CONFIG,
  USB_EVENT_SUSPEND,
  USB_EVENT_WAKEUP,
  USB_EVENT_STALL,
};

enum {
  USB_SET_INTERFACE,
  USB_GET_INTERFACE,
  USB_QUERY_INTERFACE,
};

enum DEVICE_STATE
{
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
};

void usb_lld_init (uint8_t feature);
void usb_lld_to_pmabuf (const void *src, uint16_t addr, size_t n);
void usb_lld_from_pmabuf (void *dst, uint16_t addr, size_t n);
void usb_lld_stall_tx (int ep_num);
void usb_lld_stall_rx (int ep_num);
int usb_lld_tx_data_len (int ep_num);
void usb_lld_txcpy (const void *src, int ep_num, int offset, size_t len);
void usb_lld_tx_enable (int ep_num, size_t len);
void usb_lld_write (uint8_t ep_num, const void *buf, size_t len);
int usb_lld_reply_request (const void *buf, size_t buflen,
			   struct req_args *arg);
void usb_lld_rx_enable (int ep_num);
int usb_lld_rx_data_len (int ep_num);
void usb_lld_rxcpy (uint8_t *dst, int ep_num, int offset, size_t len);
void usb_lld_reset (uint8_t feature);
void usb_lld_setup_endpoint (int ep_num, int ep_type, int ep_kind,
			     int ep_rx_addr, int ep_tx_addr,
			     int ep_rx_memory_size);
void usb_lld_set_configuration (uint8_t config);
uint8_t usb_lld_current_configuration (void);
void usb_lld_set_data_to_recv (void *p, size_t len);

void usb_lld_prepare_shutdown (void);
void usb_lld_shutdown (void);

void usb_interrupt_handler (void);
