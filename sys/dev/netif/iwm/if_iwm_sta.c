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
 * Copyright(c) 2012 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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
 * Copyright(c) 2012 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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

/* XXX common includes */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>

#include <machine/endian.h>

#include <net/bpf.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

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
#include "if_iwm_config.h"
#include "if_iwm_debug.h"
#include "if_iwm_constants.h"
#include "if_iwm_util.h"
#include "if_iwm_mac_ctxt.h"
#include "if_iwm_sta.h"

/*
 * New version of ADD_STA_sta command added new fields at the end of the
 * structure, so sending the size of the relevant API's structure is enough to
 * support both API versions.
 */
static inline int
iwm_mvm_add_sta_cmd_size(struct iwm_softc *sc)
{
#if 0
	return iwm_mvm_has_new_rx_api(mvm) ?
		sizeof(struct iwm_mvm_add_sta_cmd) :
		sizeof(struct iwm_mvm_add_sta_cmd_v7);
#else
	return sizeof(struct iwm_mvm_add_sta_cmd);
#endif
}

static int
iwm_mvm_find_free_sta_id(struct iwm_softc *sc, enum ieee80211_opmode iftype)
{
	int sta_id;
	uint32_t reserved_ids = 0;

	_Static_assert(IWM_MVM_STATION_COUNT <= 32,
	    "IWM_MVM_STATION_COUNT too large");
#if 0
	WARN_ON_ONCE(test_bit(IWL_MVM_STATUS_IN_HW_RESTART, &mvm->status));
#endif

	/* d0i3/d3 assumes the AP's sta_id (of sta vif) is 0. reserve it. */
	if (iftype != IEEE80211_M_STA)
		reserved_ids = (1 << 0);

	for (sta_id = 0; sta_id < IWM_MVM_STATION_COUNT; sta_id++) {
		if ((1U << sta_id) & reserved_ids)
			continue;

		if (!sc->fw_id_to_mac_id[sta_id])
			return sta_id;
	}
	return IWM_MVM_STATION_COUNT;
}

#if 0
/* send station add/update command to firmware */
int iwl_mvm_sta_send_to_fw(struct iwl_mvm *mvm, struct ieee80211_sta *sta,
			   bool update, unsigned int flags)
{
	struct iwl_mvm_sta *mvm_sta = iwl_mvm_sta_from_mac80211(sta);
	struct iwl_mvm_add_sta_cmd add_sta_cmd = {
		.sta_id = mvm_sta->sta_id,
		.mac_id_n_color = cpu_to_le32(mvm_sta->mac_id_n_color),
		.add_modify = update ? 1 : 0,
		.station_flags_msk = cpu_to_le32(STA_FLG_FAT_EN_MSK |
						 STA_FLG_MIMO_EN_MSK),
		.tid_disable_tx = cpu_to_le16(mvm_sta->tid_disable_agg),
	};
	int ret;
	u32 status;
	u32 agg_size = 0, mpdu_dens = 0;

	if (!update || (flags & STA_MODIFY_QUEUES)) {
		add_sta_cmd.tfd_queue_msk = cpu_to_le32(mvm_sta->tfd_queue_msk);
		memcpy(&add_sta_cmd.addr, sta->addr, ETH_ALEN);

		if (flags & STA_MODIFY_QUEUES)
			add_sta_cmd.modify_mask |= STA_MODIFY_QUEUES;
	}

	switch (sta->bandwidth) {
	case IEEE80211_STA_RX_BW_160:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_FAT_EN_160MHZ);
		/* fall through */
	case IEEE80211_STA_RX_BW_80:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_FAT_EN_80MHZ);
		/* fall through */
	case IEEE80211_STA_RX_BW_40:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_FAT_EN_40MHZ);
		/* fall through */
	case IEEE80211_STA_RX_BW_20:
		if (sta->ht_cap.ht_supported)
			add_sta_cmd.station_flags |=
				cpu_to_le32(STA_FLG_FAT_EN_20MHZ);
		break;
	}

	switch (sta->rx_nss) {
	case 1:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_MIMO_EN_SISO);
		break;
	case 2:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_MIMO_EN_MIMO2);
		break;
	case 3 ... 8:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_MIMO_EN_MIMO3);
		break;
	}

	switch (sta->smps_mode) {
	case IEEE80211_SMPS_AUTOMATIC:
	case IEEE80211_SMPS_NUM_MODES:
		WARN_ON(1);
		break;
	case IEEE80211_SMPS_STATIC:
		/* override NSS */
		add_sta_cmd.station_flags &= ~cpu_to_le32(STA_FLG_MIMO_EN_MSK);
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_MIMO_EN_SISO);
		break;
	case IEEE80211_SMPS_DYNAMIC:
		add_sta_cmd.station_flags |= cpu_to_le32(STA_FLG_RTS_MIMO_PROT);
		break;
	case IEEE80211_SMPS_OFF:
		/* nothing */
		break;
	}

	if (sta->ht_cap.ht_supported) {
		add_sta_cmd.station_flags_msk |=
			cpu_to_le32(STA_FLG_MAX_AGG_SIZE_MSK |
				    STA_FLG_AGG_MPDU_DENS_MSK);

		mpdu_dens = sta->ht_cap.ampdu_density;
	}

	if (sta->vht_cap.vht_supported) {
		agg_size = sta->vht_cap.cap &
			IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK;
		agg_size >>=
			IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT;
	} else if (sta->ht_cap.ht_supported) {
		agg_size = sta->ht_cap.ampdu_factor;
	}

	add_sta_cmd.station_flags |=
		cpu_to_le32(agg_size << STA_FLG_MAX_AGG_SIZE_SHIFT);
	add_sta_cmd.station_flags |=
		cpu_to_le32(mpdu_dens << STA_FLG_AGG_MPDU_DENS_SHIFT);

	status = ADD_STA_SUCCESS;
	ret = iwl_mvm_send_cmd_pdu_status(mvm, ADD_STA,
					  iwl_mvm_add_sta_cmd_size(mvm),
					  &add_sta_cmd, &status);
	if (ret)
		return ret;

	switch (status & IWL_ADD_STA_STATUS_MASK) {
	case ADD_STA_SUCCESS:
		IWL_DEBUG_ASSOC(mvm, "ADD_STA PASSED\n");
		break;
	default:
		ret = -EIO;
		IWL_ERR(mvm, "ADD_STA failed\n");
		break;
	}

	return ret;
}

static int iwl_mvm_sta_alloc_queue(struct iwl_mvm *mvm,
				   struct ieee80211_sta *sta, u8 ac, int tid,
				   struct ieee80211_hdr *hdr)
{
	struct iwl_mvm_sta *mvmsta = iwl_mvm_sta_from_mac80211(sta);
	struct iwl_trans_txq_scd_cfg cfg = {
		.fifo = iwl_mvm_ac_to_tx_fifo[ac],
		.sta_id = mvmsta->sta_id,
		.tid = tid,
		.frame_limit = IWL_FRAME_LIMIT,
	};
	unsigned int wdg_timeout =
		iwl_mvm_get_wd_timeout(mvm, mvmsta->vif, false, false);
	u8 mac_queue = mvmsta->vif->hw_queue[ac];
	int queue = -1;
	int ssn;
	int ret;

	lockdep_assert_held(&mvm->mutex);

	spin_lock_bh(&mvm->queue_info_lock);

	/*
	 * Non-QoS, QoS NDP and MGMT frames should go to a MGMT queue, if one
	 * exists
	 */
	if (!ieee80211_is_data_qos(hdr->frame_control) ||
	    ieee80211_is_qos_nullfunc(hdr->frame_control)) {
		queue = iwl_mvm_find_free_queue(mvm, IWL_MVM_DQA_MIN_MGMT_QUEUE,
						IWL_MVM_DQA_MAX_MGMT_QUEUE);
		if (queue >= IWL_MVM_DQA_MIN_MGMT_QUEUE)
			IWL_DEBUG_TX_QUEUES(mvm, "Found free MGMT queue #%d\n",
					    queue);

		/* If no such queue is found, we'll use a DATA queue instead */
	}

	if (queue < 0 && mvmsta->reserved_queue != IEEE80211_INVAL_HW_QUEUE) {
		queue = mvmsta->reserved_queue;
		IWL_DEBUG_TX_QUEUES(mvm, "Using reserved queue #%d\n", queue);
	}

	if (queue < 0)
		queue = iwl_mvm_find_free_queue(mvm, IWL_MVM_DQA_MIN_DATA_QUEUE,
						IWL_MVM_DQA_MAX_DATA_QUEUE);

	/*
	 * Mark TXQ as ready, even though it hasn't been fully configured yet,
	 * to make sure no one else takes it.
	 * This will allow avoiding re-acquiring the lock at the end of the
	 * configuration. On error we'll mark it back as free.
	 */
	if (queue >= 0)
		mvm->queue_info[queue].status = IWL_MVM_QUEUE_READY;

	spin_unlock_bh(&mvm->queue_info_lock);

	/* TODO: support shared queues for same RA */
	if (queue < 0)
		return -ENOSPC;

	/*
	 * Actual en/disablement of aggregations is through the ADD_STA HCMD,
	 * but for configuring the SCD to send A-MPDUs we need to mark the queue
	 * as aggregatable.
	 * Mark all DATA queues as allowing to be aggregated at some point
	 */
	cfg.aggregate = (queue >= IWL_MVM_DQA_MIN_DATA_QUEUE ||
			 queue == IWL_MVM_DQA_BSS_CLIENT_QUEUE);

	IWL_DEBUG_TX_QUEUES(mvm, "Allocating queue #%d to sta %d on tid %d\n",
			    queue, mvmsta->sta_id, tid);

	ssn = IEEE80211_SEQ_TO_SN(le16_to_cpu(hdr->seq_ctrl));
	iwl_mvm_enable_txq(mvm, queue, mac_queue, ssn, &cfg,
			   wdg_timeout);

	spin_lock_bh(&mvmsta->lock);
	mvmsta->tid_data[tid].txq_id = queue;
	mvmsta->tfd_queue_msk |= BIT(queue);

	if (mvmsta->reserved_queue == queue)
		mvmsta->reserved_queue = IEEE80211_INVAL_HW_QUEUE;
	spin_unlock_bh(&mvmsta->lock);

	ret = iwl_mvm_sta_send_to_fw(mvm, sta, true, STA_MODIFY_QUEUES);
	if (ret)
		goto out_err;

	return 0;

out_err:
	iwl_mvm_disable_txq(mvm, queue, mac_queue, tid, 0);

	return ret;
}

static inline u8 iwl_mvm_tid_to_ac_queue(int tid)
{
	if (tid == IWL_MAX_TID_COUNT)
		return IEEE80211_AC_VO; /* MGMT */

	return tid_to_mac80211_ac[tid];
}

static void iwl_mvm_tx_deferred_stream(struct iwl_mvm *mvm,
				       struct ieee80211_sta *sta, int tid)
{
	struct iwl_mvm_sta *mvmsta = iwl_mvm_sta_from_mac80211(sta);
	struct iwl_mvm_tid_data *tid_data = &mvmsta->tid_data[tid];
	struct sk_buff *skb;
	struct ieee80211_hdr *hdr;
	struct sk_buff_head deferred_tx;
	u8 mac_queue;
	bool no_queue = false; /* Marks if there is a problem with the queue */
	u8 ac;

	lockdep_assert_held(&mvm->mutex);

	skb = skb_peek(&tid_data->deferred_tx_frames);
	if (!skb)
		return;
	hdr = (void *)skb->data;

	ac = iwl_mvm_tid_to_ac_queue(tid);
	mac_queue = IEEE80211_SKB_CB(skb)->hw_queue;

	if (tid_data->txq_id == IEEE80211_INVAL_HW_QUEUE &&
	    iwl_mvm_sta_alloc_queue(mvm, sta, ac, tid, hdr)) {
		IWL_ERR(mvm,
			"Can't alloc TXQ for sta %d tid %d - dropping frame\n",
			mvmsta->sta_id, tid);

		/*
		 * Mark queue as problematic so later the deferred traffic is
		 * freed, as we can do nothing with it
		 */
		no_queue = true;
	}

	__skb_queue_head_init(&deferred_tx);

	/* Disable bottom-halves when entering TX path */
	local_bh_disable();
	spin_lock(&mvmsta->lock);
	skb_queue_splice_init(&tid_data->deferred_tx_frames, &deferred_tx);
	spin_unlock(&mvmsta->lock);

	while ((skb = __skb_dequeue(&deferred_tx)))
		if (no_queue || iwl_mvm_tx_skb(mvm, skb, sta))
			ieee80211_free_txskb(mvm->hw, skb);
	local_bh_enable();

	/* Wake queue */
	iwl_mvm_start_mac_queues(mvm, BIT(mac_queue));
}

void iwl_mvm_add_new_dqa_stream_wk(struct work_struct *wk)
{
	struct iwl_mvm *mvm = container_of(wk, struct iwl_mvm,
					   add_stream_wk);
	struct ieee80211_sta *sta;
	struct iwl_mvm_sta *mvmsta;
	unsigned long deferred_tid_traffic;
	int sta_id, tid;

	mutex_lock(&mvm->mutex);

	/* Go over all stations with deferred traffic */
	for_each_set_bit(sta_id, mvm->sta_deferred_frames,
			 IWL_MVM_STATION_COUNT) {
		clear_bit(sta_id, mvm->sta_deferred_frames);
		sta = rcu_dereference_protected(mvm->fw_id_to_mac_id[sta_id],
						lockdep_is_held(&mvm->mutex));
		if (IS_ERR_OR_NULL(sta))
			continue;

		mvmsta = iwl_mvm_sta_from_mac80211(sta);
		deferred_tid_traffic = mvmsta->deferred_traffic_tid_map;

		for_each_set_bit(tid, &deferred_tid_traffic,
				 IWL_MAX_TID_COUNT + 1)
			iwl_mvm_tx_deferred_stream(mvm, sta, tid);
	}

	mutex_unlock(&mvm->mutex);
}

static int iwl_mvm_reserve_sta_stream(struct iwl_mvm *mvm,
				      struct ieee80211_sta *sta,
				      enum nl80211_iftype vif_type)
{
	struct iwl_mvm_sta *mvmsta = iwl_mvm_sta_from_mac80211(sta);
	int queue;

	spin_lock_bh(&mvm->queue_info_lock);

	/* Make sure we have free resources for this STA */
	if (vif_type == NL80211_IFTYPE_STATION && !sta->tdls &&
	    !mvm->queue_info[IWL_MVM_DQA_BSS_CLIENT_QUEUE].hw_queue_refcount &&
	    (mvm->queue_info[IWL_MVM_DQA_BSS_CLIENT_QUEUE].status ==
	     IWL_MVM_QUEUE_FREE))
		queue = IWL_MVM_DQA_BSS_CLIENT_QUEUE;
	else
		queue = iwl_mvm_find_free_queue(mvm, IWL_MVM_DQA_MIN_DATA_QUEUE,
						IWL_MVM_DQA_MAX_DATA_QUEUE);
	if (queue < 0) {
		spin_unlock_bh(&mvm->queue_info_lock);
		IWL_ERR(mvm, "No available queues for new station\n");
		return -ENOSPC;
	}
	mvm->queue_info[queue].status = IWL_MVM_QUEUE_RESERVED;

	spin_unlock_bh(&mvm->queue_info_lock);

	mvmsta->reserved_queue = queue;

	IWL_DEBUG_TX_QUEUES(mvm, "Reserving data queue #%d for sta_id %d\n",
			    queue, mvmsta->sta_id);

	return 0;
}

int iwl_mvm_add_sta(struct iwl_mvm *mvm,
		    struct ieee80211_vif *vif,
		    struct ieee80211_sta *sta)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	struct iwl_mvm_sta *mvm_sta = iwl_mvm_sta_from_mac80211(sta);
	struct iwl_mvm_rxq_dup_data *dup_data;
	int i, ret, sta_id;

	lockdep_assert_held(&mvm->mutex);

	if (!test_bit(IWL_MVM_STATUS_IN_HW_RESTART, &mvm->status))
		sta_id = iwl_mvm_find_free_sta_id(mvm, IEEE80211_M_STA);
	else
		sta_id = mvm_sta->sta_id;

	if (sta_id == IWL_MVM_STATION_COUNT)
		return -ENOSPC;

	spin_lock_init(&mvm_sta->lock);

	mvm_sta->sta_id = sta_id;
	mvm_sta->mac_id_n_color = FW_CMD_ID_AND_COLOR(mvmvif->id,
						      mvmvif->color);
	mvm_sta->vif = vif;
	mvm_sta->max_agg_bufsize = LINK_QUAL_AGG_FRAME_LIMIT_DEF;
	mvm_sta->tx_protection = 0;
	mvm_sta->tt_tx_protection = false;

	/* HW restart, don't assume the memory has been zeroed */
	atomic_set(&mvm->pending_frames[sta_id], 0);
	mvm_sta->tid_disable_agg = 0xffff; /* No aggs at first */
	mvm_sta->tfd_queue_msk = 0;

#ifdef notyet
	/* allocate new queues for a TDLS station */
	if (sta->tdls) {
		ret = iwl_mvm_tdls_sta_init(mvm, sta);
		if (ret)
			return ret;
	} else if (!iwl_mvm_is_dqa_supported(mvm)) {
		for (i = 0; i < IEEE80211_NUM_ACS; i++)
			if (vif->hw_queue[i] != IEEE80211_INVAL_HW_QUEUE)
				mvm_sta->tfd_queue_msk |= BIT(vif->hw_queue[i]);
	}
#endif

	/* for HW restart - reset everything but the sequence number */
	for (i = 0; i <= IWL_MAX_TID_COUNT; i++) {
		u16 seq = mvm_sta->tid_data[i].seq_number;
		memset(&mvm_sta->tid_data[i], 0, sizeof(mvm_sta->tid_data[i]));
		mvm_sta->tid_data[i].seq_number = seq;

		if (!iwl_mvm_is_dqa_supported(mvm))
			continue;

		/*
		 * Mark all queues for this STA as unallocated and defer TX
		 * frames until the queue is allocated
		 */
		mvm_sta->tid_data[i].txq_id = IEEE80211_INVAL_HW_QUEUE;
		skb_queue_head_init(&mvm_sta->tid_data[i].deferred_tx_frames);
	}
	mvm_sta->deferred_traffic_tid_map = 0;
	mvm_sta->agg_tids = 0;

	if (iwl_mvm_has_new_rx_api(mvm) &&
	    !test_bit(IWL_MVM_STATUS_IN_HW_RESTART, &mvm->status)) {
		dup_data = kcalloc(mvm->trans->num_rx_queues,
				   sizeof(*dup_data),
				   GFP_KERNEL);
		if (!dup_data)
			return -ENOMEM;
		mvm_sta->dup_data = dup_data;
	}

	if (iwl_mvm_is_dqa_supported(mvm)) {
		ret = iwl_mvm_reserve_sta_stream(mvm, sta,
						 ieee80211_vif_type_p2p(vif));
		if (ret)
			goto err;
	}

	ret = iwl_mvm_sta_send_to_fw(mvm, sta, false, 0);
	if (ret)
		goto err;

	if (vif->type == NL80211_IFTYPE_STATION) {
		if (!sta->tdls) {
			WARN_ON(mvmvif->ap_sta_id != IWL_MVM_STATION_COUNT);
			mvmvif->ap_sta_id = sta_id;
		} else {
			WARN_ON(mvmvif->ap_sta_id == IWL_MVM_STATION_COUNT);
		}
	}

	rcu_assign_pointer(mvm->fw_id_to_mac_id[sta_id], sta);

	return 0;

err:
	iwl_mvm_tdls_sta_deinit(mvm, sta);
	return ret;
}

int iwl_mvm_update_sta(struct iwl_mvm *mvm,
		       struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta)
{
	return iwl_mvm_sta_send_to_fw(mvm, sta, true, 0);
}

int iwl_mvm_drain_sta(struct iwl_mvm *mvm, struct iwl_mvm_sta *mvmsta,
		      bool drain)
{
	struct iwl_mvm_add_sta_cmd cmd = {};
	int ret;
	u32 status;

	lockdep_assert_held(&mvm->mutex);

	cmd.mac_id_n_color = cpu_to_le32(mvmsta->mac_id_n_color);
	cmd.sta_id = mvmsta->sta_id;
	cmd.add_modify = STA_MODE_MODIFY;
	cmd.station_flags = drain ? cpu_to_le32(STA_FLG_DRAIN_FLOW) : 0;
	cmd.station_flags_msk = cpu_to_le32(STA_FLG_DRAIN_FLOW);

	status = ADD_STA_SUCCESS;
	ret = iwl_mvm_send_cmd_pdu_status(mvm, ADD_STA,
					  iwl_mvm_add_sta_cmd_size(mvm),
					  &cmd, &status);
	if (ret)
		return ret;

	switch (status & IWL_ADD_STA_STATUS_MASK) {
	case ADD_STA_SUCCESS:
		IWL_DEBUG_INFO(mvm, "Frames for staid %d will drained in fw\n",
			       mvmsta->sta_id);
		break;
	default:
		ret = -EIO;
		IWL_ERR(mvm, "Couldn't drain frames for staid %d\n",
			mvmsta->sta_id);
		break;
	}

	return ret;
}

/*
 * Remove a station from the FW table. Before sending the command to remove
 * the station validate that the station is indeed known to the driver (sanity
 * only).
 */
static int iwl_mvm_rm_sta_common(struct iwl_mvm *mvm, u8 sta_id)
{
	struct ieee80211_sta *sta;
	struct iwl_mvm_rm_sta_cmd rm_sta_cmd = {
		.sta_id = sta_id,
	};
	int ret;

	sta = rcu_dereference_protected(mvm->fw_id_to_mac_id[sta_id],
					lockdep_is_held(&mvm->mutex));

	/* Note: internal stations are marked as error values */
	if (!sta) {
		IWL_ERR(mvm, "Invalid station id\n");
		return -EINVAL;
	}

	ret = iwl_mvm_send_cmd_pdu(mvm, REMOVE_STA, 0,
				   sizeof(rm_sta_cmd), &rm_sta_cmd);
	if (ret) {
		IWL_ERR(mvm, "Failed to remove station. Id=%d\n", sta_id);
		return ret;
	}

	return 0;
}

void iwl_mvm_sta_drained_wk(struct work_struct *wk)
{
	struct iwl_mvm *mvm = container_of(wk, struct iwl_mvm, sta_drained_wk);
	u8 sta_id;

	/*
	 * The mutex is needed because of the SYNC cmd, but not only: if the
	 * work would run concurrently with iwl_mvm_rm_sta, it would run before
	 * iwl_mvm_rm_sta sets the station as busy, and exit. Then
	 * iwl_mvm_rm_sta would set the station as busy, and nobody will clean
	 * that later.
	 */
	mutex_lock(&mvm->mutex);

	for_each_set_bit(sta_id, mvm->sta_drained, IWL_MVM_STATION_COUNT) {
		int ret;
		struct ieee80211_sta *sta =
			rcu_dereference_protected(mvm->fw_id_to_mac_id[sta_id],
						  lockdep_is_held(&mvm->mutex));

		/*
		 * This station is in use or RCU-removed; the latter happens in
		 * managed mode, where mac80211 removes the station before we
		 * can remove it from firmware (we can only do that after the
		 * MAC is marked unassociated), and possibly while the deauth
		 * frame to disconnect from the AP is still queued. Then, the
		 * station pointer is -ENOENT when the last skb is reclaimed.
		 */
		if (!IS_ERR(sta) || PTR_ERR(sta) == -ENOENT)
			continue;

		if (PTR_ERR(sta) == -EINVAL) {
			IWL_ERR(mvm, "Drained sta %d, but it is internal?\n",
				sta_id);
			continue;
		}

		if (!sta) {
			IWL_ERR(mvm, "Drained sta %d, but it was NULL?\n",
				sta_id);
			continue;
		}

		WARN_ON(PTR_ERR(sta) != -EBUSY);
		/* This station was removed and we waited until it got drained,
		 * we can now proceed and remove it.
		 */
		ret = iwl_mvm_rm_sta_common(mvm, sta_id);
		if (ret) {
			IWL_ERR(mvm,
				"Couldn't remove sta %d after it was drained\n",
				sta_id);
			continue;
		}
		RCU_INIT_POINTER(mvm->fw_id_to_mac_id[sta_id], NULL);
		clear_bit(sta_id, mvm->sta_drained);

		if (mvm->tfd_drained[sta_id]) {
			unsigned long i, msk = mvm->tfd_drained[sta_id];

			for_each_set_bit(i, &msk, sizeof(msk) * BITS_PER_BYTE)
				iwl_mvm_disable_txq(mvm, i, i,
						    IWL_MAX_TID_COUNT, 0);

			mvm->tfd_drained[sta_id] = 0;
			IWL_DEBUG_TDLS(mvm, "Drained sta %d, with queues %ld\n",
				       sta_id, msk);
		}
	}

	mutex_unlock(&mvm->mutex);
}

static void iwl_mvm_disable_sta_queues(struct iwl_mvm *mvm,
				       struct ieee80211_vif *vif,
				       struct iwl_mvm_sta *mvm_sta)
{
	int ac;
	int i;

	lockdep_assert_held(&mvm->mutex);

	for (i = 0; i < ARRAY_SIZE(mvm_sta->tid_data); i++) {
		if (mvm_sta->tid_data[i].txq_id == IEEE80211_INVAL_HW_QUEUE)
			continue;

		ac = iwl_mvm_tid_to_ac_queue(i);
		iwl_mvm_disable_txq(mvm, mvm_sta->tid_data[i].txq_id,
				    vif->hw_queue[ac], i, 0);
		mvm_sta->tid_data[i].txq_id = IEEE80211_INVAL_HW_QUEUE;
	}
}

int iwl_mvm_rm_sta(struct iwl_mvm *mvm,
		   struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	struct iwl_mvm_sta *mvm_sta = iwl_mvm_sta_from_mac80211(sta);
	int ret;

	lockdep_assert_held(&mvm->mutex);

	if (iwl_mvm_has_new_rx_api(mvm))
		kfree(mvm_sta->dup_data);

	if (vif->type == NL80211_IFTYPE_STATION &&
	    mvmvif->ap_sta_id == mvm_sta->sta_id) {
		ret = iwl_mvm_drain_sta(mvm, mvm_sta, true);
		if (ret)
			return ret;
		/* flush its queues here since we are freeing mvm_sta */
		ret = iwl_mvm_flush_tx_path(mvm, mvm_sta->tfd_queue_msk, 0);
		if (ret)
			return ret;
		ret = iwl_trans_wait_tx_queue_empty(mvm->trans,
						    mvm_sta->tfd_queue_msk);
		if (ret)
			return ret;
		ret = iwl_mvm_drain_sta(mvm, mvm_sta, false);

		/* If DQA is supported - the queues can be disabled now */
		if (iwl_mvm_is_dqa_supported(mvm))
			iwl_mvm_disable_sta_queues(mvm, vif, mvm_sta);

		/* if we are associated - we can't remove the AP STA now */
		if (vif->bss_conf.assoc)
			return ret;

		/* unassoc - go ahead - remove the AP STA now */
		mvmvif->ap_sta_id = IWL_MVM_STATION_COUNT;

		/* clear d0i3_ap_sta_id if no longer relevant */
		if (mvm->d0i3_ap_sta_id == mvm_sta->sta_id)
			mvm->d0i3_ap_sta_id = IWL_MVM_STATION_COUNT;
	}

	/*
	 * This shouldn't happen - the TDLS channel switch should be canceled
	 * before the STA is removed.
	 */
	if (WARN_ON_ONCE(mvm->tdls_cs.peer.sta_id == mvm_sta->sta_id)) {
		mvm->tdls_cs.peer.sta_id = IWL_MVM_STATION_COUNT;
		cancel_delayed_work(&mvm->tdls_cs.dwork);
	}

	/*
	 * Make sure that the tx response code sees the station as -EBUSY and
	 * calls the drain worker.
	 */
	spin_lock_bh(&mvm_sta->lock);
	/*
	 * There are frames pending on the AC queues for this station.
	 * We need to wait until all the frames are drained...
	 */
	if (atomic_read(&mvm->pending_frames[mvm_sta->sta_id])) {
		rcu_assign_pointer(mvm->fw_id_to_mac_id[mvm_sta->sta_id],
				   ERR_PTR(-EBUSY));
		spin_unlock_bh(&mvm_sta->lock);

		/* disable TDLS sta queues on drain complete */
		if (sta->tdls) {
			mvm->tfd_drained[mvm_sta->sta_id] =
							mvm_sta->tfd_queue_msk;
			IWL_DEBUG_TDLS(mvm, "Draining TDLS sta %d\n",
				       mvm_sta->sta_id);
		}

		ret = iwl_mvm_drain_sta(mvm, mvm_sta, true);
	} else {
		spin_unlock_bh(&mvm_sta->lock);

		if (sta->tdls)
			iwl_mvm_tdls_sta_deinit(mvm, sta);

		ret = iwl_mvm_rm_sta_common(mvm, mvm_sta->sta_id);
		RCU_INIT_POINTER(mvm->fw_id_to_mac_id[mvm_sta->sta_id], NULL);
	}

	return ret;
}

int iwl_mvm_rm_sta_id(struct iwl_mvm *mvm,
		      struct ieee80211_vif *vif,
		      u8 sta_id)
{
	int ret = iwl_mvm_rm_sta_common(mvm, sta_id);

	lockdep_assert_held(&mvm->mutex);

	RCU_INIT_POINTER(mvm->fw_id_to_mac_id[sta_id], NULL);
	return ret;
}
#endif

int
iwm_mvm_allocate_int_sta(struct iwm_softc *sc, struct iwm_int_sta *sta,
	uint32_t qmask, enum ieee80211_opmode iftype)
{
#if 0
	if (!test_bit(IWL_MVM_STATUS_IN_HW_RESTART, &mvm->status)) {
#endif
		sta->sta_id = iwm_mvm_find_free_sta_id(sc, iftype);
		if (sta->sta_id == IWM_MVM_STATION_COUNT) {
			device_printf(sc->sc_dev,
			    "Failed to get free sta_id\n");
			return ENOSPC;
		}
#if 0
	}
#endif

	sta->tfd_queue_msk = qmask;

	/* put a non-NULL value so iterating over the stations won't stop */
	sc->fw_id_to_mac_id[sta->sta_id] = (void *)(-1);
	return 0;
}

static void
iwm_mvm_dealloc_int_sta(struct iwm_softc *sc, struct iwm_int_sta *sta)
{
	sc->fw_id_to_mac_id[sta->sta_id] = NULL;
	memset(sta, 0, sizeof(struct iwm_int_sta));
	sta->sta_id = IWM_MVM_STATION_COUNT;
}

int
iwm_mvm_rm_sta_id(struct iwm_softc *sc, struct ieee80211vap *vap)
{
	/* XXX wait until STA is drained */

	return iwm_mvm_rm_sta_common(sc);
}

static int
iwm_mvm_add_int_sta_common(struct iwm_softc *sc, struct iwm_int_sta *sta,
	const uint8_t *addr, uint16_t mac_id, uint16_t color)
{
	struct iwm_mvm_add_sta_cmd cmd;
	int ret;
	uint32_t status;

	memset(&cmd, 0, sizeof(cmd));
	cmd.sta_id = sta->sta_id;
	cmd.mac_id_n_color = htole32(IWM_FW_CMD_ID_AND_COLOR(mac_id, color));

	cmd.tfd_queue_msk = htole32(sta->tfd_queue_msk);
	cmd.tid_disable_tx = htole16(0xffff);

	if (addr)
		IEEE80211_ADDR_COPY(cmd.addr, addr);

	ret = iwm_mvm_send_cmd_pdu_status(sc, IWM_ADD_STA,
					  iwm_mvm_add_sta_cmd_size(sc),
					  &cmd, &status);
	if (ret)
		return ret;

	switch (status & IWM_ADD_STA_STATUS_MASK) {
	case IWM_ADD_STA_SUCCESS:
		IWM_DPRINTF(sc, IWM_DEBUG_NODE, "Internal station added.\n");
		return 0;
	default:
		ret = EIO;
		device_printf(sc->sc_dev,
		    "Add internal station failed, status=0x%x\n", status);
		break;
	}
	return ret;
}

int
iwm_mvm_add_aux_sta(struct iwm_softc *sc)
{
	int ret;

	/* Map Aux queue to fifo - needs to happen before adding Aux station */
        ret = iwm_enable_txq(sc, 0, IWM_MVM_AUX_QUEUE, IWM_MVM_TX_FIFO_MCAST);

	/* Allocate aux station and assign to it the aux queue */
	ret = iwm_mvm_allocate_int_sta(sc, &sc->sc_aux_sta,
	    (1 << IWM_MVM_AUX_QUEUE), IEEE80211_OPMODE_MAX);
	if (ret)
		return ret;

	ret = iwm_mvm_add_int_sta_common(sc, &sc->sc_aux_sta, NULL,
					 IWM_MAC_INDEX_AUX, 0);

	if (ret)
		iwm_mvm_dealloc_int_sta(sc, &sc->sc_aux_sta);
	return ret;
}

void iwm_mvm_del_aux_sta(struct iwm_softc *sc)
{
	iwm_mvm_dealloc_int_sta(sc, &sc->sc_aux_sta);
}

#if 0
const u8 tid_to_mac80211_ac[] = {
	IEEE80211_AC_BE,
	IEEE80211_AC_BK,
	IEEE80211_AC_BK,
	IEEE80211_AC_BE,
	IEEE80211_AC_VI,
	IEEE80211_AC_VI,
	IEEE80211_AC_VO,
	IEEE80211_AC_VO,
};

static const u8 tid_to_ucode_ac[] = {
	AC_BE,
	AC_BK,
	AC_BK,
	AC_BE,
	AC_VI,
	AC_VI,
	AC_VO,
	AC_VO,
};

static int iwl_mvm_set_fw_key_idx(struct iwl_mvm *mvm)
{
	int i, max = -1, max_offs = -1;

	/* Pick the unused key offset with the highest 'deleted'
	 * counter. Every time a key is deleted, all the counters
	 * are incremented and the one that was just deleted is
	 * reset to zero. Thus, the highest counter is the one
	 * that was deleted longest ago. Pick that one.
	 */
	for (i = 0; i < STA_KEY_MAX_NUM; i++) {
		if (test_bit(i, mvm->fw_key_table))
			continue;
		if (mvm->fw_key_deleted[i] > max) {
			max = mvm->fw_key_deleted[i];
			max_offs = i;
		}
	}

	if (max_offs < 0)
		return STA_KEY_IDX_INVALID;

	return max_offs;
}

static struct iwl_mvm_sta *iwl_mvm_get_key_sta(struct iwl_mvm *mvm,
					       struct ieee80211_vif *vif,
					       struct ieee80211_sta *sta)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

	if (sta)
		return iwl_mvm_sta_from_mac80211(sta);

	/*
	 * The device expects GTKs for station interfaces to be
	 * installed as GTKs for the AP station. If we have no
	 * station ID, then use AP's station ID.
	 */
	if (vif->type == NL80211_IFTYPE_STATION &&
	    mvmvif->ap_sta_id != IWL_MVM_STATION_COUNT) {
		u8 sta_id = mvmvif->ap_sta_id;

		sta = sc->fw_id_to_mac_id[sta_id];

		/*
		 * It is possible that the 'sta' parameter is NULL,
		 * for example when a GTK is removed - the sta_id will then
		 * be the AP ID, and no station was passed by mac80211.
		 */
		if (IS_ERR_OR_NULL(sta))
			return NULL;

		return iwl_mvm_sta_from_mac80211(sta);
	}

	return NULL;
}

static int iwl_mvm_send_sta_key(struct iwl_mvm *mvm,
				struct iwl_mvm_sta *mvm_sta,
				struct ieee80211_key_conf *keyconf, bool mcast,
				u32 tkip_iv32, u16 *tkip_p1k, u32 cmd_flags,
				u8 key_offset)
{
	struct iwl_mvm_add_sta_key_cmd cmd = {};
	__le16 key_flags;
	int ret;
	u32 status;
	u16 keyidx;
	int i;
	u8 sta_id = mvm_sta->sta_id;

	keyidx = (keyconf->keyidx << STA_KEY_FLG_KEYID_POS) &
		 STA_KEY_FLG_KEYID_MSK;
	key_flags = cpu_to_le16(keyidx);
	key_flags |= cpu_to_le16(STA_KEY_FLG_WEP_KEY_MAP);

	switch (keyconf->cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		key_flags |= cpu_to_le16(STA_KEY_FLG_TKIP);
		cmd.tkip_rx_tsc_byte2 = tkip_iv32;
		for (i = 0; i < 5; i++)
			cmd.tkip_rx_ttak[i] = cpu_to_le16(tkip_p1k[i]);
		memcpy(cmd.key, keyconf->key, keyconf->keylen);
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key_flags |= cpu_to_le16(STA_KEY_FLG_CCM);
		memcpy(cmd.key, keyconf->key, keyconf->keylen);
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		key_flags |= cpu_to_le16(STA_KEY_FLG_WEP_13BYTES);
		/* fall through */
	case WLAN_CIPHER_SUITE_WEP40:
		key_flags |= cpu_to_le16(STA_KEY_FLG_WEP);
		memcpy(cmd.key + 3, keyconf->key, keyconf->keylen);
		break;
	default:
		key_flags |= cpu_to_le16(STA_KEY_FLG_EXT);
		memcpy(cmd.key, keyconf->key, keyconf->keylen);
	}

	if (mcast)
		key_flags |= cpu_to_le16(STA_KEY_MULTICAST);

	cmd.key_offset = key_offset;
	cmd.key_flags = key_flags;
	cmd.sta_id = sta_id;

	status = ADD_STA_SUCCESS;
	if (cmd_flags & CMD_ASYNC)
		ret =  iwl_mvm_send_cmd_pdu(mvm, ADD_STA_KEY, CMD_ASYNC,
					    sizeof(cmd), &cmd);
	else
		ret = iwl_mvm_send_cmd_pdu_status(mvm, ADD_STA_KEY, sizeof(cmd),
						  &cmd, &status);

	switch (status) {
	case ADD_STA_SUCCESS:
		IWL_DEBUG_WEP(mvm, "MODIFY_STA: set dynamic key passed\n");
		break;
	default:
		ret = -EIO;
		IWL_ERR(mvm, "MODIFY_STA: set dynamic key failed\n");
		break;
	}

	return ret;
}

static int iwl_mvm_send_sta_igtk(struct iwl_mvm *mvm,
				 struct ieee80211_key_conf *keyconf,
				 u8 sta_id, bool remove_key)
{
	struct iwl_mvm_mgmt_mcast_key_cmd igtk_cmd = {};

	/* verify the key details match the required command's expectations */
	if (WARN_ON((keyconf->cipher != WLAN_CIPHER_SUITE_AES_CMAC) ||
		    (keyconf->flags & IEEE80211_KEY_FLAG_PAIRWISE) ||
		    (keyconf->keyidx != 4 && keyconf->keyidx != 5)))
		return -EINVAL;

	igtk_cmd.key_id = cpu_to_le32(keyconf->keyidx);
	igtk_cmd.sta_id = cpu_to_le32(sta_id);

	if (remove_key) {
		igtk_cmd.ctrl_flags |= cpu_to_le32(STA_KEY_NOT_VALID);
	} else {
		struct ieee80211_key_seq seq;
		const u8 *pn;

		switch (keyconf->cipher) {
		case WLAN_CIPHER_SUITE_AES_CMAC:
			igtk_cmd.ctrl_flags |= cpu_to_le32(STA_KEY_FLG_CCM);
			break;
		default:
			return -EINVAL;
		}

		memcpy(igtk_cmd.IGTK, keyconf->key, keyconf->keylen);
		ieee80211_get_key_rx_seq(keyconf, 0, &seq);
		pn = seq.aes_cmac.pn;
		igtk_cmd.receive_seq_cnt = cpu_to_le64(((u64) pn[5] << 0) |
						       ((u64) pn[4] << 8) |
						       ((u64) pn[3] << 16) |
						       ((u64) pn[2] << 24) |
						       ((u64) pn[1] << 32) |
						       ((u64) pn[0] << 40));
	}

	IWL_DEBUG_INFO(mvm, "%s igtk for sta %u\n",
		       remove_key ? "removing" : "installing",
		       igtk_cmd.sta_id);

	return iwl_mvm_send_cmd_pdu(mvm, MGMT_MCAST_KEY, 0,
				    sizeof(igtk_cmd), &igtk_cmd);
}


static inline uint8_t *
iwm_mvm_get_mac_addr(struct iwm_softc *sc,
				       struct ieee80211_vif *vif,
				       struct ieee80211_sta *sta)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

	if (sta)
		return sta->addr;

	if (vif->type == IEEE80211_M_STA &&
	    mvmvif->ap_sta_id != IWM_MVM_STATION_COUNT) {
		u8 sta_id = mvmvif->ap_sta_id;
		sta = sc->fw_id_to_mac_id[sta_id];
		return sta->addr;
	}

	return NULL;
}

static int __iwl_mvm_set_sta_key(struct iwl_mvm *mvm,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 struct ieee80211_key_conf *keyconf,
				 u8 key_offset,
				 bool mcast)
{
	struct iwl_mvm_sta *mvm_sta = iwl_mvm_sta_from_mac80211(sta);
	int ret;
	const u8 *addr;
	struct ieee80211_key_seq seq;
	u16 p1k[5];

	switch (keyconf->cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		addr = iwl_mvm_get_mac_addr(mvm, vif, sta);
		/* get phase 1 key from mac80211 */
		ieee80211_get_key_rx_seq(keyconf, 0, &seq);
		ieee80211_get_tkip_rx_p1k(keyconf, addr, seq.tkip.iv32, p1k);
		ret = iwl_mvm_send_sta_key(mvm, mvm_sta, keyconf, mcast,
					   seq.tkip.iv32, p1k, 0, key_offset);
		break;
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		ret = iwl_mvm_send_sta_key(mvm, mvm_sta, keyconf, mcast,
					   0, NULL, 0, key_offset);
		break;
	default:
		ret = iwl_mvm_send_sta_key(mvm, mvm_sta, keyconf, mcast,
					   0, NULL, 0, key_offset);
	}

	return ret;
}

static int __iwl_mvm_remove_sta_key(struct iwl_mvm *mvm, u8 sta_id,
				    struct ieee80211_key_conf *keyconf,
				    bool mcast)
{
	struct iwl_mvm_add_sta_key_cmd cmd = {};
	__le16 key_flags;
	int ret;
	u32 status;

	key_flags = cpu_to_le16((keyconf->keyidx << STA_KEY_FLG_KEYID_POS) &
				 STA_KEY_FLG_KEYID_MSK);
	key_flags |= cpu_to_le16(STA_KEY_FLG_NO_ENC | STA_KEY_FLG_WEP_KEY_MAP);
	key_flags |= cpu_to_le16(STA_KEY_NOT_VALID);

	if (mcast)
		key_flags |= cpu_to_le16(STA_KEY_MULTICAST);

	cmd.key_flags = key_flags;
	cmd.key_offset = keyconf->hw_key_idx;
	cmd.sta_id = sta_id;

	status = ADD_STA_SUCCESS;
	ret = iwl_mvm_send_cmd_pdu_status(mvm, ADD_STA_KEY, sizeof(cmd),
					  &cmd, &status);

	switch (status) {
	case ADD_STA_SUCCESS:
		IWL_DEBUG_WEP(mvm, "MODIFY_STA: remove sta key passed\n");
		break;
	default:
		ret = -EIO;
		IWL_ERR(mvm, "MODIFY_STA: remove sta key failed\n");
		break;
	}

	return ret;
}

int iwl_mvm_set_sta_key(struct iwl_mvm *mvm,
			struct ieee80211_vif *vif,
			struct ieee80211_sta *sta,
			struct ieee80211_key_conf *keyconf,
			u8 key_offset)
{
	bool mcast = !(keyconf->flags & IEEE80211_KEY_FLAG_PAIRWISE);
	struct iwl_mvm_sta *mvm_sta;
	u8 sta_id;
	int ret;
	static const u8 __maybe_unused zero_addr[ETH_ALEN] = {0};

	lockdep_assert_held(&mvm->mutex);

	/* Get the station id from the mvm local station table */
	mvm_sta = iwl_mvm_get_key_sta(mvm, vif, sta);
	if (!mvm_sta) {
		IWL_ERR(mvm, "Failed to find station\n");
		return -EINVAL;
	}
	sta_id = mvm_sta->sta_id;

	if (keyconf->cipher == WLAN_CIPHER_SUITE_AES_CMAC) {
		ret = iwl_mvm_send_sta_igtk(mvm, keyconf, sta_id, false);
		goto end;
	}

	/*
	 * It is possible that the 'sta' parameter is NULL, and thus
	 * there is a need to retrieve  the sta from the local station table.
	 */
	if (!sta) {
		sta = rcu_dereference_protected(mvm->fw_id_to_mac_id[sta_id],
						lockdep_is_held(&mvm->mutex));
		if (IS_ERR_OR_NULL(sta)) {
			IWL_ERR(mvm, "Invalid station id\n");
			return -EINVAL;
		}
	}

	if (WARN_ON_ONCE(iwl_mvm_sta_from_mac80211(sta)->vif != vif))
		return -EINVAL;

	/* If the key_offset is not pre-assigned, we need to find a
	 * new offset to use.  In normal cases, the offset is not
	 * pre-assigned, but during HW_RESTART we want to reuse the
	 * same indices, so we pass them when this function is called.
	 *
	 * In D3 entry, we need to hardcoded the indices (because the
	 * firmware hardcodes the PTK offset to 0).  In this case, we
	 * need to make sure we don't overwrite the hw_key_idx in the
	 * keyconf structure, because otherwise we cannot configure
	 * the original ones back when resuming.
	 */
	if (key_offset == STA_KEY_IDX_INVALID) {
		key_offset  = iwl_mvm_set_fw_key_idx(mvm);
		if (key_offset == STA_KEY_IDX_INVALID)
			return -ENOSPC;
		keyconf->hw_key_idx = key_offset;
	}

	ret = __iwl_mvm_set_sta_key(mvm, vif, sta, keyconf, key_offset, mcast);
	if (ret)
		goto end;

	/*
	 * For WEP, the same key is used for multicast and unicast. Upload it
	 * again, using the same key offset, and now pointing the other one
	 * to the same key slot (offset).
	 * If this fails, remove the original as well.
	 */
	if (keyconf->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	    keyconf->cipher == WLAN_CIPHER_SUITE_WEP104) {
		ret = __iwl_mvm_set_sta_key(mvm, vif, sta, keyconf,
					    key_offset, !mcast);
		if (ret) {
			__iwl_mvm_remove_sta_key(mvm, sta_id, keyconf, mcast);
			goto end;
		}
	}

	__set_bit(key_offset, mvm->fw_key_table);

end:
	IWL_DEBUG_WEP(mvm, "key: cipher=%x len=%d idx=%d sta=%pM ret=%d\n",
		      keyconf->cipher, keyconf->keylen, keyconf->keyidx,
		      sta ? sta->addr : zero_addr, ret);
	return ret;
}

int iwl_mvm_remove_sta_key(struct iwl_mvm *mvm,
			   struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta,
			   struct ieee80211_key_conf *keyconf)
{
	bool mcast = !(keyconf->flags & IEEE80211_KEY_FLAG_PAIRWISE);
	struct iwl_mvm_sta *mvm_sta;
	u8 sta_id = IWL_MVM_STATION_COUNT;
	int ret, i;

	lockdep_assert_held(&mvm->mutex);

	/* Get the station from the mvm local station table */
	mvm_sta = iwl_mvm_get_key_sta(mvm, vif, sta);

	IWL_DEBUG_WEP(mvm, "mvm remove dynamic key: idx=%d sta=%d\n",
		      keyconf->keyidx, sta_id);

	if (keyconf->cipher == WLAN_CIPHER_SUITE_AES_CMAC)
		return iwl_mvm_send_sta_igtk(mvm, keyconf, sta_id, true);

	if (!__test_and_clear_bit(keyconf->hw_key_idx, mvm->fw_key_table)) {
		IWL_ERR(mvm, "offset %d not used in fw key table.\n",
			keyconf->hw_key_idx);
		return -ENOENT;
	}

	/* track which key was deleted last */
	for (i = 0; i < STA_KEY_MAX_NUM; i++) {
		if (mvm->fw_key_deleted[i] < U8_MAX)
			mvm->fw_key_deleted[i]++;
	}
	mvm->fw_key_deleted[keyconf->hw_key_idx] = 0;

	if (!mvm_sta) {
		IWL_DEBUG_WEP(mvm, "station non-existent, early return.\n");
		return 0;
	}

	sta_id = mvm_sta->sta_id;

	ret = __iwl_mvm_remove_sta_key(mvm, sta_id, keyconf, mcast);
	if (ret)
		return ret;

	/* delete WEP key twice to get rid of (now useless) offset */
	if (keyconf->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	    keyconf->cipher == WLAN_CIPHER_SUITE_WEP104)
		ret = __iwl_mvm_remove_sta_key(mvm, sta_id, keyconf, !mcast);

	return ret;
}

void iwl_mvm_update_tkip_key(struct iwl_mvm *mvm,
			     struct ieee80211_vif *vif,
			     struct ieee80211_key_conf *keyconf,
			     struct ieee80211_sta *sta, u32 iv32,
			     u16 *phase1key)
{
	struct iwl_mvm_sta *mvm_sta;
	bool mcast = !(keyconf->flags & IEEE80211_KEY_FLAG_PAIRWISE);

	rcu_read_lock();

	mvm_sta = iwl_mvm_get_key_sta(mvm, vif, sta);
	if (WARN_ON_ONCE(!mvm_sta))
		goto unlock;
	iwl_mvm_send_sta_key(mvm, mvm_sta, keyconf, mcast,
			     iv32, phase1key, CMD_ASYNC, keyconf->hw_key_idx);

 unlock:
	rcu_read_unlock();
}
#endif
