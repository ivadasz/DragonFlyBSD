#ifndef _HID_IOCTL_H_
#define	_HID_IOCTL_H_

#include <sys/ioccom.h>

struct hid_report_request {
	uint16_t len;
	uint8_t kind;
	uint8_t id;
	unsigned char *report;
} __packed;

/* Specify report_desc == NULL, to only retrieve the length. */
struct hid_report_desc {
	uint16_t len;
	char *report_desc;
};

#define UHID_GET_REPORT_DESC	_IOWR('H', 0, struct hid_report_desc)
#if 0
#define UHID_SET_IMMED		_IOW ('H', 1, int)
#endif
#define UHID_GET_REPORT		_IOWR('H', 2, struct hid_report_request)
#define UHID_SET_REPORT		_IOW ('H', 3, struct hid_report_request)
#if 0
#define UHID_GET_REPORT_ID	_IOR ('H', 4, int)
#endif

#endif					/* _HID_IOCTL_H_ */
