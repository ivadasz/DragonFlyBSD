/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 4.7.3 (tag id d7f6728f57e3ecbb7ef34eb7d9f564d514775d75)
 *
 ******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2016        Intel Deutschland GmbH
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
 * Copyright(c) 2016        Intel Deutschland GmbH
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
 *
 *****************************************************************************/

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
#include "if_iwm_constants.h"
#include "if_iwm_debug.h"
#include "if_iwm_util.h"
#include "if_iwm_quota.h"

#define QUOTA_100	IWM_MVM_MAX_QUOTA

struct iwm_mvm_quota_iterator_data {
	int n_interfaces[IWM_MAX_BINDINGS];
	int colors[IWM_MAX_BINDINGS];
	struct ieee80211vap *disabled_vif;
};

static void
iwm_mvm_quota_iterator(void *_data, struct ieee80211vap *vap)
{
	struct iwm_mvm_quota_iterator_data *data = _data;
	struct iwm_vap *ivp = IWM_VAP(vap);
	uint16_t id;

	/* skip disabled interfaces here immediately */
	if (vap == data->disabled_vif)
		return;

	if (!ivp->phy_ctxt)
		return;

	/* currently, PHY ID == binding ID */
	id = ivp->phy_ctxt->id;

	_Static_assert(IWM_NUM_PHY_CTX <= IWM_MAX_BINDINGS,
	    "need at least one binding per PHY");

	KKASSERT(id < IWM_MAX_BINDINGS);

#if 0
	if (!IWM_NODE(ivp->iv_bss)->in_assoc)
		return;
#endif

	if (data->colors[id] < 0)
		data->colors[id] = ivp->phy_ctxt->color;
	else
		KKASSERT(data->colors[id] == mvmvif->phy_ctxt->color);

	data->n_interfaces[id]++;
}

int
iwm_mvm_update_quotas(struct iwm_softc *sc, boolean_t force_update,
	struct ieee80211vap *disabled_vif)
{
	struct iwm_time_quota_cmd cmd = {};
	int i, idx, err, num_active_macs, quota, quota_rem;
	struct iwm_mvm_quota_iterator_data data = {
		.n_interfaces = {},
		.colors = { -1, -1, -1, -1 },
		.disabled_vif = disabled_vif,
	};
	struct iwm_time_quota_cmd *last = &sc->last_quota_cmd;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap;
	boolean_t send = FALSE;

	/* iterator data above must match */
	_Static_assert(IWM_MAX_BINDINGS == 4,
	    "iterator data above must match");

	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		iwm_mvm_quota_iterator(&data, vap);
	}

	/*
	 * The FW's scheduling session consists of
	 * IWM_MVM_MAX_QUOTA fragments. Divide these fragments
	 * equally between all the bindings that require quota
	 */
	num_active_macs = 0;
	for (i = 0; i < IWM_MAX_BINDINGS; i++) {
		cmd.quotas[i].id_and_color = htole32(IWM_FW_CTXT_INVALID);
		num_active_macs += data.n_interfaces[i];
	}

	if (num_active_macs) {
		/*
		 * There are 0 or more than 1 low latency bindings, or all the
		 * data interfaces belong to the single low latency binding.
		 * Split the quota equally between the data interfaces.
		 */
		quota = QUOTA_100 / num_active_macs;
		quota_rem = QUOTA_100 % num_active_macs;
	} else {
		/* values don't really matter - won't be used */
		quota = 0;
		quota_rem = 0;
	}

	for (idx = 0, i = 0; i < IWM_MAX_BINDINGS; i++) {
		if (data.colors[i] < 0)
			continue;

		cmd.quotas[idx].id_and_color =
			htole32(IWM_FW_CMD_ID_AND_COLOR(i, data.colors[i]));

		if (data.n_interfaces[i] <= 0)
			cmd.quotas[idx].quota = htole32(0);
		else
			cmd.quotas[idx].quota =
				htole32(quota * data.n_interfaces[i]);

		if (le32toh(cmd.quotas[idx].quota) > QUOTA_100) {
			device_printf(sc->sc_dev,
			    "Binding=%d, quota=%u > max=%u\n",
			    idx, le32toh(cmd.quotas[idx].quota), QUOTA_100);
		}

		cmd.quotas[idx].max_duration = htole32(0);

		idx++;
	}

	/* Give the remainder of the session to the first data binding */
	for (i = 0; i < IWM_MAX_BINDINGS; i++) {
		if (le32toh(cmd.quotas[i].quota) != 0) {
			cmd.quotas[i].quota =
			    htole32(le32toh(cmd.quotas[i].quota) + quota_rem);
			IWM_DPRINTF(sc, IWM_DEBUG_CMD,
			    "quota: giving remainder of %d to binding %d\n",
			    quota_rem, i);
			break;
		}
	}

	/* check that we have non-zero quota for all valid bindings */
	for (i = 0; i < IWM_MAX_BINDINGS; i++) {
		if (cmd.quotas[i].id_and_color != last->quotas[i].id_and_color)
			send = TRUE;
		if (cmd.quotas[i].max_duration != last->quotas[i].max_duration)
			send = TRUE;
		if (abs((int)le32toh(cmd.quotas[i].quota) -
			(int)le32toh(last->quotas[i].quota))
						> IWM_MVM_QUOTA_THRESHOLD)
			send = TRUE;
		if (cmd.quotas[i].id_and_color == htole32(IWM_FW_CTXT_INVALID))
			continue;
		if (cmd.quotas[i].quota == 0)
		    device_printf(sc->sc_dev, "zero quota on binding %d\n", i);
	}

	if (!send && !force_update) {
		/* don't send a practically unchanged command, the firmware has
		 * to re-initialize a lot of state and that can have an adverse
		 * impact on it
		 */
		return 0;
	}

	err = iwm_mvm_send_cmd_pdu(sc, IWM_TIME_QUOTA_CMD, 0, sizeof(cmd),
	    &cmd);

	if (err)
		device_printf(sc->sc_dev, "Failed to send quota: %d\n", err);
	else
		sc->last_quota_cmd = cmd;
	return err;
}
