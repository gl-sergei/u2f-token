#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

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
  DEVICE_RECIPIENT,     /* Recipient device    */
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

void usb_cb_device_reset (void);
void usb_cb_ctrl_write_finish (uint8_t req, uint8_t req_no,
			       uint16_t value, uint16_t index, uint16_t len);
int usb_cb_setup (uint8_t req, uint8_t req_no, uint16_t value,
		  uint16_t index, uint16_t len);
int usb_cb_get_descriptor (uint8_t desc_type, uint16_t index, uint16_t value);
int usb_cb_handle_event (uint8_t event_type, uint16_t value);
int usb_cb_interface (uint8_t cmd, uint16_t interface, uint16_t value);

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

extern uint32_t bDeviceState;
extern const uint8_t usb_initial_feature;

#define STM32_USB_IRQ_PRIORITY     11

extern void usb_lld_init (uint8_t feature);

extern void usb_lld_to_pmabuf (const void *src, uint16_t addr, size_t n);

extern void usb_lld_from_pmabuf (void *dst, uint16_t addr, size_t n);

extern void usb_lld_stall_tx (int ep_num);

extern void usb_lld_stall_rx (int ep_num);

extern int usb_lld_tx_data_len (int ep_num);

extern void usb_lld_txcpy (const void *src, int ep_num, int offset, size_t len);

extern void usb_lld_tx_enable (int ep_num, size_t len);

extern void usb_lld_write (uint8_t ep_num, const void *buf, size_t len);

extern void usb_lld_rx_enable (int ep_num);

extern int usb_lld_rx_data_len (int ep_num);

extern void usb_lld_rxcpy (uint8_t *dst, int ep_num, int offset, size_t len);

extern void usb_lld_reset (void);

extern void usb_lld_setup_endpoint (int ep_num, int ep_type, int ep_kind,
				    int ep_rx_addr, int ep_tx_addr,
				    int ep_rx_memory_size);

extern void usb_lld_set_configuration (uint8_t config);

extern uint8_t usb_lld_current_configuration (void);

extern void usb_lld_set_feature (uint8_t feature);

extern void usb_lld_set_data_to_send (const void *p, size_t len);

extern inline void usb_lld_set_data_to_recv (void *p, size_t len)
{
  usb_lld_set_data_to_send ((const void *)p, len);
}

extern void usb_lld_prepare_shutdown (void);
extern void usb_lld_shutdown (void);

extern void usb_interrupt_handler (void);
