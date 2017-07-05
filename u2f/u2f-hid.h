#ifndef __U2F_U2F_HID_H__

#define __U2F_U2F_HID_H__

struct usb_hid;
struct u2f_hid;

struct u2f_hid *
u2f_hid_open (struct usb_hid *hid);

#endif
