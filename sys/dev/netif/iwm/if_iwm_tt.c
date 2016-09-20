/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 4.7.3 (tag id d7f6728f57e3ecbb7ef34eb7d9f564d514775d75)
 *
 ***********************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2015 - 2016 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2015 - 2016 Intel Deutschland GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/linker.h>

#include <machine/endian.h>

#include <bus/pci/pcivar.h>
#include <bus/pci/pcireg.h>

#include <net/bpf.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ifq_var.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <netproto/802_11/ieee80211_var.h>
#include <netproto/802_11/ieee80211_regdomain.h>
#include <netproto/802_11/ieee80211_ratectl.h>
#include <netproto/802_11/ieee80211_radiotap.h>

#include "if_iwmreg.h"
#include "if_iwmvar.h"
#include "if_iwm_debug.h"
#include "if_iwm_util.h"
#include "if_iwm_tt.h"

static int
iwm_mvm_temp_notif_parse(struct iwm_softc *sc, struct iwm_rx_packet *pkt)
{
	struct iwm_dts_measurement_notif_v1 *notif_v1;
	int len = iwm_rx_packet_payload_len(pkt);
	int temp;

	/* we can use notif_v1 only, because v2 only adds an additional
	 * parameter, which is not used in this function.
	*/
	if (len < sizeof(*notif_v1)) {
		device_printf(sc->sc_dev,
		    "Invalid DTS_MEASUREMENT_NOTIFICATION\n");
		return -EINVAL;
	}

	notif_v1 = (void *)pkt->data;

	temp = le32toh(notif_v1->temp);

	/* shouldn't be negative, but since it's s32, make sure it isn't */
	if (temp < 0) {
		device_printf(sc->sc_dev, "%s: temp is negative\n",
		    __func__);
		temp = 0;
	}

	IWM_DPRINTF(sc, IWM_DEBUG_TEMP,
	    "DTS_MEASUREMENT_NOTIFICATION - %d\n", temp);

	return temp;
}

#ifdef notyet
static boolean_t
iwl_mvm_temp_notif_wait(struct iwm_softc *sc,
	struct iwm_rx_packet *pkt, void *data)
{
	int *temp = data;
	int ret;

	ret = iwm_mvm_temp_notif_parse(sc, pkt);
	if (ret < 0)
		return TRUE;

	*temp = ret;

	return TRUE;
}
#endif

static inline boolean_t
iwm_mvm_is_tt_in_fw(struct iwm_softc *sc)
{
	/* these two TLV are redundant since the responsibility to CT-kill by
	 * FW happens only after we send at least one command of
	 * temperature THs report.
	 */
	return fw_has_capa(&sc->ucode_capa,
		     IWM_UCODE_TLV_CAPA_CT_KILL_BY_FW) &&
	       fw_has_capa(&sc->ucode_capa,
		     IWM_UCODE_TLV_CAPA_TEMP_THS_REPORT_SUPPORT);
}

void
iwm_mvm_temp_notif(struct iwm_softc *sc, struct iwm_rx_packet *pkt)
{
	struct iwm_dts_measurement_notif_v2 *notif_v2;
	int len = iwm_rx_packet_payload_len(pkt);
	int temp;
	uint32_t ths_crossed;

#ifdef notyet
	/* the notification is handled synchronously in ctkill, so skip here */
	if (test_bit(IWL_MVM_STATUS_HW_CTKILL, &mvm->status))
		return;
#endif

	temp = iwm_mvm_temp_notif_parse(sc, pkt);

	if (!iwm_mvm_is_tt_in_fw(sc)) {
#ifdef notyet
		if (temp >= 0)
			iwm_mvm_tt_temp_changed(sc, temp);
#endif
		return;
	}

	if (len < sizeof(*notif_v2)) {
	        device_printf(sc->sc_dev,
		    "Invalid DTS_MEASUREMENT_NOTIFICATION\n");
		return;
	}

	notif_v2 = (void *)pkt->data;
	ths_crossed = le32toh(notif_v2->threshold_idx);

	/* 0xFF in ths_crossed means the notification is not related
	 * to a trip, so we can ignore it here.
	 */
	if (ths_crossed == 0xFF)
		return;

	IWM_DPRINTF(sc, IWM_DEBUG_TEMP, "Temp = %d Threshold crossed = %d\n",
	    temp, ths_crossed);
}
