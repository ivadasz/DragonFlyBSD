#ifndef BUS_HID_HIDVAR_H
#define BUS_HID_HIDVAR_H

enum hid_bootproto {
	HID_BOOTPROTO_OTHER,
	HID_BOOTPROTO_KEYBOARD,
	HID_BOOTPROTO_MOUSE,
};

enum hid_device_ivars {
	HID_IVAR_BOOTPROTO,
};

__BUS_ACCESSOR(hid, bootproto, HID, BOOTPROTO, int)

#endif /* BUS_HID_HIDVAR_H */
