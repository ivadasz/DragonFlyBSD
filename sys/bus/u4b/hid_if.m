#-
# Copyright (c) 2016 The DragonFly Project.  All rights reserved.
#
# This code is derived from software contributed to The DragonFly Project
# by Imre Vadász <imre@vdsz.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of The DragonFly Project nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific, prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

#include <sys/bus.h>

#include <bus/u4b/hid_common.h>

INTERFACE hid;

# Retrieve HID descriptor.
METHOD void get_descriptor {
	device_t dev;
	char **descp;
	uint16_t *sizep;
};

# Set NULL handler to stop handling input reports.
METHOD void set_handler {
	device_t dev;
	hid_input_handler_t input;
	hid_output_handler_t output;
	void *arg;
};

# Activates input from hardware device.
METHOD void start_read {
	device_t dev;
};

# Deactivates input from hardware device.
# WARNING: Further input reports may still be delivered via the handler
#          method after deactivating input with stop().
METHOD void stop_read {
	device_t dev;
};

# SETIDLE
METHOD void setidle {
	device_t dev;
	uint8_t duration;
	uint8_t id;
}

# SET_REPORT
# Currently has exactly one transfer in flight at a time. Successful transfers
# call the output_handler callback with a nonzero length, cancelled transfers
# call the callback with a zero length.
METHOD void set_report {
	device_t dev;
	uint8_t id;
	uint8_t *buf;
	uint16_t len;
}

# SET_FEATURE
# Sets the feature report, blocking.
METHOD int set_feature {
	device_t dev;
	uint8_t id;
	uint8_t *buf;
	uint16_t len;
}

# GET_REPORT
# Gets the input report, or feature report. Blocking.
# The caller should figure out how long the buffer should be for the report id.
METHOD int get_report {
	device_t dev;
	uint8_t id;
	uint8_t *buf;
	uint16_t len;
	int type;
}
