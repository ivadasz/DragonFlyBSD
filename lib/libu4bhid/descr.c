/*	$NetBSD: descr.c,v 1.9 2000/09/24 02:13:24 augustss Exp $	*/

/*
 * Copyright (c) 1999 Lennart Augustsson <augustss@netbsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: head/lib/libusbhid/descr.c 240762 2012-09-20 18:56:27Z mav $
 */

#include <sys/types.h>

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <bus/hid/hid_ioctl.h>

#include "usbhid.h"
#include "usbvar.h"

int
hid_set_immed(int fd, int enable)
{
#if 0
	int ret;
	ret = ioctl(fd, UHID_SET_IMMED, &enable);
	return (ret);
#else
	return (0);
#endif
}

int
hid_get_report_id(int fd)
{
	report_desc_t rep;
	hid_data_t d;
	hid_item_t h;
	int kindset;
	int temp = -1;
#if 0
	int ret;
#endif

	if ((rep = hid_get_report_desc(fd)) == NULL)
		goto use_ioctl;
	kindset = 1 << hid_input | 1 << hid_output | 1 << hid_feature;
	for (d = hid_start_parse(rep, kindset, -1); hid_get_item(d, &h); ) {
		/* Return the first report ID we met. */
		if (h.report_ID != 0) {
			temp = h.report_ID;
			break;
		}
	}
	hid_end_parse(d);
	hid_dispose_report_desc(rep);

	if (temp > 0)
		return (temp);

use_ioctl:
#if 0
	ret = ioctl(fd, USB_GET_REPORT_ID, &temp);
	ret = temp;

	return (ret);
#else
	return (0);
#endif
}

report_desc_t
hid_get_report_desc(int fd)
{
	struct hid_report_desc desc;
	report_desc_t rep;
	void *data;

	memset(&desc, 0, sizeof(desc));

	/* get actual length first */
	desc.report_desc = NULL;
	desc.len = 0;
	if (ioctl(fd, UHID_GET_REPORT_DESC, &desc) < 0)
		return (NULL);

	data = malloc(desc.len);
	if (data == NULL)
		return (NULL);

	/* fetch actual descriptor */
	desc.report_desc = data;
	if (ioctl(fd, UHID_GET_REPORT_DESC, &desc) < 0) {
		/* could not read descriptor */
		free(data);
		return (NULL);
	}

	/* sanity check */
	if (desc.len < 1) {
		/* invalid report descriptor */
		free(data);
		return (NULL);
	}

	/* check END_COLLECTION */
	if (((unsigned char *)data)[desc.len -1] != 0xC0) {
		/* invalid end byte */
		free(data);
		return (NULL);
	}

	rep = hid_use_report_desc(data, desc.len);

	free(data);

	return (rep);
}

report_desc_t
hid_use_report_desc(unsigned char *data, unsigned int size)
{
	report_desc_t r;

	r = malloc(sizeof(*r) + size);
	if (r == 0) {
		errno = ENOMEM;
		return (NULL);
	}
	r->size = size;
	memcpy(r->data, data, size);
	return (r);
}

void
hid_dispose_report_desc(report_desc_t r)
{

	free(r);
}
