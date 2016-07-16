/*-
 * Copyright (c) 2002-2008 Sam Leffler, Errno Consulting
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * IEEE 802.11 scanning support.
 */
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <netproto/802_11/ieee80211_var.h>

#include "if_iwm_scan_fw.h"

#include <net/bpf.h>

struct scan_state {
	struct ieee80211_scan_state base; /* public state */

	struct task	ss_scan_start;	/* scan start */
	struct task	ss_scan_check;  /* scan check */
	u_int		ss_iflags;	/* flags used internally */
#define	ISCAN_DISCARD	0x0001		/* discard rx'd frames */
#define	ISCAN_CANCEL	0x0002		/* cancel current scan */
#define	ISCAN_ABORT	0x0004		/* end the scan immediately */
#define	ISCAN_RUNNING	0x0008		/* scan was started */
};
#define SCAN_PRIVATE(ss)	((struct scan_state *) ss)

static	void scan_signal(struct ieee80211_scan_state *, int);
static	void scan_signal_locked(struct ieee80211_scan_state *, int);
static	void scan_start(void *, int);
static	void scan_check_task(void *, int);
static	void scan_end(struct ieee80211_scan_state *, int);
static	void scan_done(struct ieee80211_scan_state *, int);

static void
scan_signal(struct ieee80211_scan_state *ss, int iflags)
{
	struct ieee80211com *ic = ss->ss_ic;

	IEEE80211_UNLOCK_ASSERT(ic);

	IEEE80211_LOCK(ic);
	scan_signal_locked(ss, iflags);
	IEEE80211_UNLOCK(ic);
}

static void
scan_signal_locked(struct ieee80211_scan_state *ss, int iflags)
{
	struct scan_state *ss_priv = SCAN_PRIVATE(ss);
	struct ieee80211com *ic = ss->ss_ic;

	IEEE80211_LOCK_ASSERT(ic);

	ss_priv->ss_iflags |= iflags;
	if (ss_priv->ss_iflags & ISCAN_RUNNING)
		ieee80211_runtask(ic, &SCAN_PRIVATE(ss)->ss_scan_check);
}

static void
iwm_fwscan_detach(struct ieee80211com *ic)
{
	struct ieee80211_scan_state *ss = ic->ic_scan;

	if (ss != NULL) {
		scan_signal(ss, ISCAN_ABORT);
		ieee80211_draintask(ic, &SCAN_PRIVATE(ss)->ss_scan_start);
		ieee80211_draintask(ic, &SCAN_PRIVATE(ss)->ss_scan_check);
		KASSERT((ic->ic_flags & IEEE80211_F_SCAN) == 0,
                    ("scan still running"));

		/*
		 * For now, do the ss_ops detach here rather
		 * than ieee80211_scan_detach().
		 *
		 * I'll figure out how to cleanly split things up
		 * at a later date.
		 */
		if (ss->ss_ops != NULL) {
			ss->ss_ops->scan_detach(ss);
			ss->ss_ops = NULL;
		}
		ic->ic_scan = NULL;
		kfree(SCAN_PRIVATE(ss), M_DEVBUF);
	}
}

static void
iwm_fwscan_vattach(struct ieee80211vap *vap)
{
}

static void
iwm_fwscan_vdetach(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	IEEE80211_LOCK_ASSERT(ic);

	if (ss != NULL && ss->ss_vap == vap &&
	    (ic->ic_flags & IEEE80211_F_SCAN))
		scan_signal_locked(ss, ISCAN_ABORT);
}

static void
iwm_fwscan_set_scan_duration(struct ieee80211vap *vap, u_int duration)
{
	/* XXX ignore for now */
}

static int
iwm_fwscan_start_scan_locked(const struct ieee80211_scanner *scan,
    struct ieee80211vap *vap, int flags,
    u_int nssid, const struct ieee80211_scan_ssid ssids[])
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	IEEE80211_LOCK_ASSERT(ic);

	if (ic->ic_flags & IEEE80211_F_CSAPENDING) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: scan inhibited by pending channel change\n", __func__);
	} else if ((ic->ic_flags & IEEE80211_F_SCAN) == 0) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: %s scan, desired mode %s, %s%s%s%s%s%s\n"
		    , __func__
		    , flags & IEEE80211_SCAN_ACTIVE ? "active" : "passive"
		    , ieee80211_phymode_name[vap->iv_des_mode]
		    , flags & IEEE80211_SCAN_FLUSH ? "flush" : "append"
		    , flags & IEEE80211_SCAN_NOPICK ? ", nopick" : ""
		    , flags & IEEE80211_SCAN_NOJOIN ? ", nojoin" : ""
		    , flags & IEEE80211_SCAN_NOBCAST ? ", nobcast" : ""
		    , flags & IEEE80211_SCAN_PICK1ST ? ", pick1st" : ""
		    , flags & IEEE80211_SCAN_ONCE ? ", once" : ""
		);

		ieee80211_scan_update_locked(vap, scan);
		if (ss->ss_ops != NULL) {
			if ((flags & IEEE80211_SCAN_NOSSID) == 0)
				ieee80211_scan_copy_ssid(vap, ss, nssid, ssids);

			/* NB: top 4 bits for internal use */
			ss->ss_flags = flags & 0xfff;
			if (ss->ss_flags & IEEE80211_SCAN_ACTIVE)
				vap->iv_stats.is_scan_active++;
			else
				vap->iv_stats.is_scan_passive++;
			if (flags & IEEE80211_SCAN_FLUSH)
				ss->ss_ops->scan_flush(ss);
			if (flags & IEEE80211_SCAN_BGSCAN)
				ic->ic_flags_ext |= IEEE80211_FEXT_BGSCAN;

			ss->ss_next = 0;
			/* Dummy values for ss_mindwell and ss_maxdwell ... */
			ss->ss_mindwell = 0;
			ss->ss_maxdwell = 0;
			/* NB: scan_start must be before the scan runtask */
			ss->ss_ops->scan_start(ss, vap);
#ifdef IEEE80211_DEBUG
			if (ieee80211_msg_scan(vap))
				ieee80211_scan_dump(ss);
#endif /* IEEE80211_DEBUG */
			ic->ic_flags |= IEEE80211_F_SCAN;

			/* Start scan task */
			ieee80211_runtask(ic, &SCAN_PRIVATE(ss)->ss_scan_start);
		}
		return 1;
	} else {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: %s scan already in progress\n", __func__,
		    ss->ss_flags & IEEE80211_SCAN_ACTIVE ? "active" : "passive");
	}
	return 0;
}

static int
iwm_fwscan_start_scan(const struct ieee80211_scanner *scan,
    struct ieee80211vap *vap, int flags,
    u_int duration, u_int mindwell, u_int maxdwell,
    u_int nssid, const struct ieee80211_scan_ssid ssids[])
{
	struct ieee80211com *ic = vap->iv_ic;
	int result;

	IEEE80211_UNLOCK_ASSERT(ic);

	IEEE80211_LOCK(ic);
	/* XXX ignore scan parameters for now */
	result = iwm_fwscan_start_scan_locked(scan, vap, flags,
	    nssid, ssids);
	IEEE80211_UNLOCK(ic);

	return result;
}

static void
scan_start(void *arg, int pending)
{
	struct ieee80211_scan_state *ss = (struct ieee80211_scan_state *) arg;
	struct scan_state *ss_priv = SCAN_PRIVATE(ss);
	struct ieee80211vap *vap = ss->ss_vap;
	struct ieee80211com *ic = ss->ss_ic;

	IEEE80211_LOCK(ic);
	if (vap == NULL || (ic->ic_flags & IEEE80211_F_SCAN) == 0 ||
	    (ss_priv->ss_iflags & ISCAN_ABORT)) {
		/* Cancelled before we started */
		scan_done(ss, 0);
		return;
	}

	if (ss->ss_next == ss->ss_last) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
			"%s: no channels to scan\n", __func__);
		scan_done(ss, 1);
		return;
	}
	ss_priv->ss_iflags |= ISCAN_RUNNING;
	IEEE80211_UNLOCK(ic);

	ic->ic_scan_start(ic);		/* notify driver */

	scan_check_task(ss, 0);
}

static void
scan_check_task(void *arg, int pending)
{
	struct ieee80211_scan_state *ss = arg;
	struct scan_state *ss_priv = SCAN_PRIVATE(ss);
	struct ieee80211com *ic = ss->ss_ic;
	int scandone;

	IEEE80211_LOCK(ic);
	scandone = (ss->ss_next >= ss->ss_last) ||
	    (ss_priv->ss_iflags & ISCAN_CANCEL) != 0;

	IEEE80211_DPRINTF(ss->ss_vap, IEEE80211_MSG_SCAN,
	    "%s: loop start; scandone=%d\n",
	    __func__,
	    scandone);

	if (scandone || (ss->ss_flags & IEEE80211_SCAN_GOTPICK) ||
	    (ss_priv->ss_iflags & ISCAN_ABORT)) {
		ss_priv->ss_iflags &= ~ISCAN_RUNNING;
		scan_end(ss, scandone);
		return;
	}

	IEEE80211_DPRINTF(ss->ss_vap, IEEE80211_MSG_SCAN, "%s: waiting\n",
	    __func__);
	IEEE80211_UNLOCK(ic);
}

static int
iwm_fwscan_check_scan(const struct ieee80211_scanner *scan,
    struct ieee80211vap *vap, int flags,
    u_int duration, u_int mindwell, u_int maxdwell,
    u_int nssid, const struct ieee80211_scan_ssid ssids[])
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	int result;

	IEEE80211_LOCK_ASSERT(ic);

	if (ss->ss_ops != NULL) {
		/* XXX verify ss_ops matches vap->iv_opmode */
		if ((flags & IEEE80211_SCAN_NOSSID) == 0) {
			/*
			 * Update the ssid list and mark flags so if
			 * we call start_scan it doesn't duplicate work.
			 */
			ieee80211_scan_copy_ssid(vap, ss, nssid, ssids);
			flags |= IEEE80211_SCAN_NOSSID;
		}
		if ((ic->ic_flags & IEEE80211_F_SCAN) == 0 &&
		    (flags & IEEE80211_SCAN_FLUSH) == 0 &&
		    ieee80211_time_before(ticks, ic->ic_lastscan + vap->iv_scanvalid)) {
			/*
			 * We're not currently scanning and the cache is
			 * deemed hot enough to consult.  Lock out others
			 * by marking IEEE80211_F_SCAN while we decide if
			 * something is already in the scan cache we can
			 * use.  Also discard any frames that might come
			 * in while temporarily marked as scanning.
			 */
			SCAN_PRIVATE(ss)->ss_iflags |= ISCAN_DISCARD;
			ic->ic_flags |= IEEE80211_F_SCAN;

			/* NB: need to use supplied flags in check */
			ss->ss_flags = flags & 0xff;
			result = ss->ss_ops->scan_end(ss, vap);

			ic->ic_flags &= ~IEEE80211_F_SCAN;
			SCAN_PRIVATE(ss)->ss_iflags &= ~ISCAN_DISCARD;
			if (result) {
				ieee80211_notify_scan_done(vap);
				return 1;
			}
		}
	}
	result = iwm_fwscan_start_scan_locked(scan, vap, flags, nssid, ssids);

	return result;
}

/*
 * Restart a previous scan.  If the previous scan completed
 * then we start again using the existing channel list.
 */
static int
iwm_fwscan_bg_scan(const struct ieee80211_scanner *scan,
    struct ieee80211vap *vap, int flags)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	/* XXX assert unlocked? */
	// IEEE80211_UNLOCK_ASSERT(ic);

	IEEE80211_LOCK(ic);
	if ((ic->ic_flags & IEEE80211_F_SCAN) == 0) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: %s scan, ticks %u\n", __func__,
		    ss->ss_flags & IEEE80211_SCAN_ACTIVE ? "active" : "passive",
		    ticks);

		ieee80211_scan_update_locked(vap, scan);
		if (ss->ss_ops != NULL) {
			ss->ss_vap = vap;
			/*
			 * A background scan does not select a new sta; it
			 * just refreshes the scan cache.
			 */
			ss->ss_flags |= IEEE80211_SCAN_NOPICK
				     |  IEEE80211_SCAN_BGSCAN
				     |  flags
				     ;
			/* if previous scan completed, restart */
			if (ss->ss_next >= ss->ss_last) {
				if (ss->ss_flags & IEEE80211_SCAN_ACTIVE)
					vap->iv_stats.is_scan_active++;
				else
					vap->iv_stats.is_scan_passive++;
				/*
				 * NB: beware of the scan cache being flushed;
				 *     if the channel list is empty use the
				 *     scan_start method to populate it.
				 */
				ss->ss_next = 0;
				if (ss->ss_last != 0) {
					ss->ss_ops->scan_restart(ss, vap);
				} else {
					ss->ss_ops->scan_start(ss, vap);
#ifdef IEEE80211_DEBUG
					if (ieee80211_msg_scan(vap))
						ieee80211_scan_dump(ss);
#endif /* IEEE80211_DEBUG */
				}
			}
			ic->ic_flags |= IEEE80211_F_SCAN;
			ic->ic_flags_ext |= IEEE80211_FEXT_BGSCAN;
			ieee80211_runtask(ic,
			    &SCAN_PRIVATE(ss)->ss_scan_start);
		} else {
			/* XXX msg+stat */
		}
	} else {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: %s scan already in progress\n", __func__,
		    ss->ss_flags & IEEE80211_SCAN_ACTIVE ? "active" : "passive");
	}
	IEEE80211_UNLOCK(ic);

	/* NB: racey, does it matter? */
	return (ic->ic_flags & IEEE80211_F_SCAN);
}

static void
cancel_scan(struct ieee80211vap *vap, int any, const char *func)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	IEEE80211_LOCK(ic);
	if ((ic->ic_flags & IEEE80211_F_SCAN) &&
	    (any || ss->ss_vap == vap) &&
	    (SCAN_PRIVATE(ss)->ss_iflags & ISCAN_CANCEL) == 0) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: cancel %s scan\n", func,
		    ss->ss_flags & IEEE80211_SCAN_ACTIVE ?
			"active" : "passive");

		/* clear bg scan NOPICK */
		ss->ss_flags &= ~IEEE80211_SCAN_NOPICK;
		/* mark cancel request and wake up the scan task */
		scan_signal_locked(ss, ISCAN_CANCEL);
	} else {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: called; F_SCAN=%d, vap=%s, CANCEL=%d\n",
			func,
			!! (ic->ic_flags & IEEE80211_F_SCAN),
			(ss->ss_vap == vap ? "match" : "nomatch"),
			!! (SCAN_PRIVATE(ss)->ss_iflags & ISCAN_CANCEL));
	}
	IEEE80211_UNLOCK(ic);
}

/*
 * Cancel any scan currently going on for the specified vap.
 */
static void
iwm_fwscan_cancel_scan(struct ieee80211vap *vap)
{
	cancel_scan(vap, 0, __func__);
}

/*
 * Cancel any scan currently going on.
 */
static void
iwm_fwscan_cancel_anyscan(struct ieee80211vap *vap)
{
	cancel_scan(vap, 1, __func__);
}

static void
iwm_fwscan_next(struct ieee80211vap *vap)
{
	struct ieee80211_scan_state *ss = vap->iv_ic->ic_scan;

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "%s: called\n", __func__);

	/* wake up the scan task */
	scan_signal(ss, 0);
}

static void
iwm_fwscan_done(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	IEEE80211_LOCK_ASSERT(ic);

	scan_signal_locked(ss, 0);
}

static void
iwm_fwscan_probe_curchan(struct ieee80211vap *vap, int force)
{
}

static void
scan_end(struct ieee80211_scan_state *ss, int scandone)
{
	struct scan_state *ss_priv = SCAN_PRIVATE(ss);
	struct ieee80211vap *vap = ss->ss_vap;
	struct ieee80211com *ic = ss->ss_ic;

	IEEE80211_LOCK_ASSERT(ic);

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "%s: out\n", __func__);

	if (ss_priv->ss_iflags & ISCAN_ABORT) {
		scan_done(ss, scandone);
		return;
	}

	IEEE80211_UNLOCK(ic);
	ic->ic_scan_end(ic);		/* notify driver */
	IEEE80211_LOCK(ic);
	/* XXX scan state can change! Re-validate scan state! */

	/*
	 * Since a cancellation may have occurred during one of the
	 * driver calls (whilst unlocked), update scandone.
	 */
	if (scandone == 0 && (ss_priv->ss_iflags & ISCAN_CANCEL) != 0) {
		/* XXX printf? */
		if_printf(vap->iv_ifp,
		    "%s: OOPS! scan cancelled during driver call (1)!\n",
		    __func__);
		scandone = 1;
	}

	/*
	 * Record scan complete time.  Note that we also do
	 * this when canceled so any background scan will
	 * not be restarted for a while.
	 */
	if (scandone)
		ic->ic_lastscan = ticks;
	/* clear internal flags and any indication of a pick */
	ss->ss_flags &= ~IEEE80211_SCAN_GOTPICK;

	/*
	 * If not canceled and scan completed, do post-processing.
	 * If the callback function returns 0, then it wants to
	 * continue/restart scanning.  Unfortunately we needed to
	 * notify the driver to end the scan above to avoid having
	 * rx frames alter the scan candidate list.
	 */
	if ((ss_priv->ss_iflags & ISCAN_CANCEL) == 0 &&
	    !ss->ss_ops->scan_end(ss, vap) &&
	    (ss->ss_flags & IEEE80211_SCAN_ONCE) == 0) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
		    "%s: done, restart [ticks %u]\n",
		    __func__, ticks);
		ss->ss_next = 0;	/* reset to beginning */
		if (ss->ss_flags & IEEE80211_SCAN_ACTIVE)
			vap->iv_stats.is_scan_active++;
		else
			vap->iv_stats.is_scan_passive++;

		ss->ss_ops->scan_restart(ss, vap);	/* XXX? */
		ieee80211_runtask(ic, &ss_priv->ss_scan_start);
		IEEE80211_UNLOCK(ic);
		return;
	}

	/* past here, scandone is ``true'' if not in bg mode */
	if ((ss->ss_flags & IEEE80211_SCAN_BGSCAN) == 0)
		scandone = 1;

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
	    "%s: %s, [ticks %u]\n",
	    __func__, scandone ? "done" : "stopped", ticks);

	scan_done(ss, scandone);
}

static void
scan_done(struct ieee80211_scan_state *ss, int scandone)
{
	struct scan_state *ss_priv = SCAN_PRIVATE(ss);
	struct ieee80211com *ic = ss->ss_ic;
	struct ieee80211vap *vap = ss->ss_vap;

	IEEE80211_LOCK_ASSERT(ic);

	ic->ic_flags &= ~IEEE80211_F_SCAN;

	/*
	 * If this scan was prematurely terminated because it is
	 * a background scan then don't notify the ap.
	 */
	if (scandone) {
		if (ss->ss_next >= ss->ss_last)
			ic->ic_flags_ext &= ~IEEE80211_FEXT_BGSCAN;
		ieee80211_notify_scan_done(vap);
	}
	ss_priv->ss_iflags &= ~(ISCAN_CANCEL|ISCAN_ABORT);
	ss->ss_flags &= ~(IEEE80211_SCAN_ONCE | IEEE80211_SCAN_PICK1ST);
	IEEE80211_UNLOCK(ic);
}

/*
 * Process a beacon or probe response frame.
 */
static void
iwm_fwscan_add_scan(struct ieee80211vap *vap,
    struct ieee80211_channel *curchan,
    const struct ieee80211_scanparams *sp,
    const struct ieee80211_frame *wh,
    int subtype, int rssi, int noise)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;

	/* XXX locking */

	if (SCAN_PRIVATE(ss)->ss_iflags & ISCAN_DISCARD)
		return;
#ifdef IEEE80211_DEBUG
	if (ieee80211_msg_scan(vap) && (ic->ic_flags & IEEE80211_F_SCAN))
		ieee80211_scan_dump_probe_beacon(subtype, 1, wh->i_addr2, sp, rssi);
#endif
	if (ss->ss_ops != NULL)
		ss->ss_ops->scan_add(ss, curchan, sp, wh, subtype, rssi, noise);
}

static struct ieee80211_scan_methods iwm_fwscan_methods = {
	.sc_attach = iwm_fwscan_attach,
	.sc_detach = iwm_fwscan_detach,
	.sc_vattach = iwm_fwscan_vattach,
	.sc_vdetach = iwm_fwscan_vdetach,
	.sc_set_scan_duration = iwm_fwscan_set_scan_duration,
	.sc_start_scan = iwm_fwscan_start_scan,
	.sc_check_scan = iwm_fwscan_check_scan,
	.sc_bg_scan = iwm_fwscan_bg_scan,
	.sc_cancel_scan = iwm_fwscan_cancel_scan,
	.sc_cancel_anyscan = iwm_fwscan_cancel_anyscan,
	.sc_scan_next = iwm_fwscan_next,
	.sc_scan_done = iwm_fwscan_done,
	.sc_scan_probe_curchan = iwm_fwscan_probe_curchan,
	.sc_add_scan = iwm_fwscan_add_scan
};

void
iwm_fwscan_attach(struct ieee80211com *ic)
{
	struct scan_state *ss;

	/* Allocate initial scan state */
#if defined(__DragonFly__)
	ss = (struct scan_state *) kmalloc(sizeof(struct scan_state),
		M_DEVBUF, M_INTWAIT | M_ZERO);
#else
	ss = (struct scan_state *) malloc(sizeof(struct scan_state),
		M_DEVBUF, M_NOWAIT | M_ZERO);
#endif
	if (ss == NULL)
		return;

	TASK_INIT(&ss->ss_scan_start, 0, scan_start, ss);
	TASK_INIT(&ss->ss_scan_check, 0, scan_check_task, ss);

	ic->ic_scan_methods = &iwm_fwscan_methods;
	ic->ic_scan = &ss->base;
	ss->base.ss_ic = ic;
}
