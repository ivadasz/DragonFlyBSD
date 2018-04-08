#ifndef BUS_HID_HIDVAR_H
#define BUS_HID_HIDVAR_H

enum hid_protocol {
	HID_UIPROTO_OTHER,
	HID_UIPROTO_BOOT_KEYBOARD,
	HID_UIPROTO_MOUSE,
};

enum hid_device_ivars {
	HID_IVAR_BUSTYPE,
	HID_IVAR_PRODUCT,
	HID_IVAR_VENDOR,
	HID_IVAR_REVISION,
	HID_IVAR_SERIAL,
	HID_IVAR_INTERVAL_MS,
	HID_IVAR_UIPROTO,
};

enum hid_bus_type {
	HID_BUS_USB, HID_BUS_IIC,
};

__BUS_ACCESSOR(hid, bustype, HID, BUSTYPE, int)
__BUS_ACCESSOR(hid, product, HID, PRODUCT, uint16_t)
__BUS_ACCESSOR(hid, vendor, HID, VENDOR, uint16_t)
__BUS_ACCESSOR(hid, revision, HID, REVISION, uint16_t)
__BUS_ACCESSOR(hid, serial, HID, SERIAL, const char *)
__BUS_ACCESSOR(hid, interval_ms, HID, INTERVAL_MS, uint16_t)
__BUS_ACCESSOR(hid, uiproto, HID, UIPROTO, int)

#endif /* BUS_HID_HIDVAR_H */
