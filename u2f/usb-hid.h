#ifndef __U2F_USB_HID_H__

#define __U2F_USB_HID_H__

struct usb_hid;

struct usb_hid *
hid_open (void);

int
hid_send (struct usb_hid *usb_hid, uint8_t *buf, uint16_t len);

int
hid_recv (struct usb_hid *usb_hid, uint8_t *buf, uint16_t len, uint32_t timeout);

#endif
