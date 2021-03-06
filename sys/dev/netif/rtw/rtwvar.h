/*
 * Copyright (c) 2004, 2005 David Young.  All rights reserved.
 *
 * Driver for the Realtek RTL8180 802.11 MAC/BBP by David Young.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of David Young may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY David Young ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL David
 * Young BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * $NetBSD: rtwvar.h,v 1.28 2006/02/16 20:17:16 perry Exp $
 * $DragonFly: src/sys/dev/netif/rtw/rtwvar.h,v 1.5 2008/01/15 09:01:13 sephe Exp $
 */

#ifndef _DEV_IC_RTWVAR_H_
#define	_DEV_IC_RTWVAR_H_

/*
 * 802.11 frame duration definitions.
 */

struct rtw_duration {
	uint16_t        d_rts_dur;
	uint16_t        d_data_dur;
	uint16_t        d_plcp_len;
	uint8_t         d_residue;	/* unused octets in time slot */
};

#ifdef RTW_DEBUG
#define	RTW_DEBUG_TUNE		0x0000001
#define	RTW_DEBUG_PKTFILT	0x0000002
#define	RTW_DEBUG_XMIT		0x0000004
#define	RTW_DEBUG_XMIT_DESC	0x0000008
#define	RTW_DEBUG_NODE		0x0000010
#define	RTW_DEBUG_PWR		0x0000020
#define	RTW_DEBUG_ATTACH	0x0000040
#define	RTW_DEBUG_REGDUMP	0x0000080
#define	RTW_DEBUG_ACCESS	0x0000100
#define	RTW_DEBUG_RESET		0x0000200
#define	RTW_DEBUG_INIT		0x0000400
#define	RTW_DEBUG_IOSTATE	0x0000800
#define	RTW_DEBUG_RECV		0x0001000
#define	RTW_DEBUG_RECV_DESC	0x0002000
#define	RTW_DEBUG_IO_KICK	0x0004000
#define	RTW_DEBUG_INTR		0x0008000
#define	RTW_DEBUG_PHY		0x0010000
#define	RTW_DEBUG_PHYIO		0x0020000
#define	RTW_DEBUG_PHYBITIO	0x0040000
#define	RTW_DEBUG_TIMEOUT	0x0080000
#define	RTW_DEBUG_BUGS		0x0100000
#define	RTW_DEBUG_BEACON	0x0200000
#define	RTW_DEBUG_LED		0x0400000
#define	RTW_DEBUG_KEY		0x0800000
#define	RTW_DEBUG_XMIT_RSRC	0x1000000
#define	RTW_DEBUG_OACTIVE	0x2000000
#define	RTW_DEBUG_MAX		0x3ffffff

extern int	rtw_debug;
#define RTW_DPRINTF(__flags, __x)	\
	if ((rtw_debug & (__flags)) != 0) kprintf __x
#define	DPRINTF(__sc, __flags, __x)				\
	if (((__sc)->sc_if.if_flags & IFF_DEBUG) != 0)	\
		RTW_DPRINTF(__flags, __x)
#define	RTW_PRINT_REGS(__regs, __dvname, __where)	\
	rtw_print_regs((__regs), (__dvname), (__where))
#else /* RTW_DEBUG */
#define RTW_DPRINTF(__flags, __x)
#define	DPRINTF(__sc, __flags, __x)
#define	RTW_PRINT_REGS(__regs, __dvname, __where)
#endif /* RTW_DEBUG */

enum rtw_locale {
	RTW_LOCALE_USA	= 0,
	RTW_LOCALE_EUROPE,
	RTW_LOCALE_JAPAN,
	RTW_LOCALE_UNKNOWN
};

enum rtw_rfchipid {
	RTW_RFCHIPID_RESERVED	= 0,
	RTW_RFCHIPID_INTERSIL	= 1,
	RTW_RFCHIPID_RFMD	= 2,
	RTW_RFCHIPID_PHILIPS	= 3,
	RTW_RFCHIPID_MAXIM	= 4,
	RTW_RFCHIPID_GCT	= 5
};

/* sc_flags */
#define RTW_F_ENABLED		0x00000001	/* chip is enabled */
#define RTW_F_DIGPHY		0x00000002	/* digital PHY */
#define RTW_F_DFLANTB		0x00000004	/* B antenna is default */
#define RTW_F_ANTDIV		0x00000010	/* h/w antenna diversity */
#define RTW_F_9356SROM		0x00000020	/* 93c56 SROM */
#define RTW_F_SLEEP		0x00000040	/* chip is asleep */
#define RTW_F_INVALID		0x00000080	/* chip is absent */
#define	RTW_F_DK_VALID		0x00000100	/* keys in DK0-DK3 are valid */
#define	RTW_C_RXWEP_40		0x00000200	/* h/w decrypts 40-bit WEP */
#define	RTW_C_RXWEP_104		0x00000400	/* h/w decrypts 104-bit WEP */
	/* all PHY flags */
#define RTW_F_ALLPHY		(RTW_F_DIGPHY|RTW_F_DFLANTB|RTW_F_ANTDIV)

enum rtw_access {
	RTW_ACCESS_NONE		= 0,
	RTW_ACCESS_CONFIG	= 1,
	RTW_ACCESS_ANAPARM	= 2
};

struct rtw_regs {
	bus_space_tag_t		r_bt;
	bus_space_handle_t	r_bh;
	enum rtw_access		r_access;
	int			r_type;
	int			r_rid;
	struct resource		*r_res;
};

#define RTW_SR_GET(sr, ofs) \
    (((sr)->sr_content[(ofs)/2] >> (((ofs) % 2 == 0) ? 0 : 8)) & 0xff)

#define RTW_SR_GET16(sr, ofs) \
    (RTW_SR_GET((sr), (ofs)) | (RTW_SR_GET((sr), (ofs) + 1) << 8))

struct rtw_srom {
	uint16_t	*sr_content;
	uint16_t	sr_size;
};

struct rtw_rxsoft {
	struct mbuf	*rs_mbuf;
	bus_dmamap_t	rs_dmamap;
	bus_addr_t	rs_phyaddr;
};

struct rtw_txsoft {
	STAILQ_ENTRY(rtw_txsoft)	ts_q;
	struct mbuf			*ts_mbuf;
	bus_dmamap_t			ts_dmamap;
	struct ieee80211_node		*ts_ni;		/* destination node */
	int				ts_ratectl;
	int				ts_rateidx;
	u_int				ts_first;	/* 1st hw descriptor */
	u_int				ts_last;	/* last hw descriptor */
	struct rtw_duration		ts_d0;
	struct rtw_duration		ts_dn;
};

#define RTW_NTXPRI	4	/* number of Tx priorities */
#define RTW_TXPRILO	0
#define RTW_TXPRIMD	1
#define RTW_TXPRIHI	2
#define RTW_TXPRIBCN	3	/* beacon priority */

#define RTW_MAXPKTSEGS	64	/* Max 64 segments per Tx packet */

#define CASSERT(cond, complaint)	\
	complaint[(cond) ? 0 : -1] = complaint[(cond) ? 0 : -1]

/*
 * Note well: the descriptor rings must begin on RTW_DESC_ALIGNMENT
 * boundaries.  I allocate them consecutively from one buffer, so
 * just round up.
 */
#define RTW_TXQLENLO	64	/* low-priority queue length */
#define RTW_TXQLENMD	64	/* medium-priority */
#define RTW_TXQLENHI	64	/* high-priority */
#define RTW_TXQLENBCN	8	/* beacon */

#define RTW_NTXDESCLO	RTW_TXQLENLO
#define RTW_NTXDESCMD	RTW_TXQLENMD
#define RTW_NTXDESCHI	RTW_TXQLENHI
#define RTW_NTXDESCBCN	RTW_TXQLENBCN

#define RTW_NTXDESCTOTAL	(RTW_NTXDESCLO + RTW_NTXDESCMD +	\
				 RTW_NTXDESCHI + RTW_NTXDESCBCN)

#define RTW_RXQLEN	64

struct rtw_rxdesc_blk {
	int			rdb_ndesc;
	int			rdb_next;

	bus_dma_tag_t		rdb_dmat;
	bus_dmamap_t		rdb_dmamap;
	struct rtw_rxdesc	*rdb_desc;

	uint32_t		rdb_base;	/* XXX bus_addr_t */
};

struct rtw_txdesc_blk {
	int			tdb_ndesc;
	int			tdb_next;
	int			tdb_nfree;

	bus_dma_tag_t		tdb_dmat;
	bus_dmamap_t		tdb_dmamap;
	struct rtw_txdesc	*tdb_desc;

	bus_size_t		tdb_basereg;
	uint32_t		tdb_base;	/* XXX bus_addr_t */
};

#define RTW_NEXT_IDX(__htc, __idx)	(((__idx) + 1) % (__htc)->tdb_ndesc)

#define RTW_NEXT_DESC(__htc, __idx)	\
	((__htc)->tdb_base +		\
	 sizeof(struct rtw_txdesc) * RTW_NEXT_IDX((__htc), (__idx)))

STAILQ_HEAD(rtw_txq, rtw_txsoft);

struct rtw_txsoft_blk {
	/* dirty/free s/w descriptors */
	struct rtw_txq		tsb_dirtyq;
	struct rtw_txq		tsb_freeq;
	u_int			tsb_ndesc;
	int			tsb_tx_timer;
	struct rtw_txsoft	*tsb_desc;
	uint8_t			tsb_poll;
};

struct rtw_descs {
	struct rtw_txdesc	hd_txlo[RTW_NTXDESCLO];
	struct rtw_txdesc	hd_txmd[RTW_NTXDESCMD];
	struct rtw_txdesc	hd_txhi[RTW_NTXDESCMD];
	struct rtw_rxdesc	hd_rx[RTW_RXQLEN];
	struct rtw_txdesc	hd_bcn[RTW_NTXDESCBCN];
};
#define RTW_DESC_OFFSET(ring, i)	offsetof(struct rtw_descs, ring[i])
#define RTW_RING_OFFSET(ring)		RTW_DESC_OFFSET(ring, 0)
#define RTW_RING_BASE(sc, ring)		((sc)->sc_desc_physaddr +	\
					 RTW_RING_OFFSET(ring))

/* Radio capture format for RTL8180. */

#define RTW_RX_RADIOTAP_PRESENT					\
	((1 << IEEE80211_RADIOTAP_TSFT)			|	\
	 (1 << IEEE80211_RADIOTAP_FLAGS)		|	\
	 (1 << IEEE80211_RADIOTAP_RATE)			|	\
	 (1 << IEEE80211_RADIOTAP_CHANNEL)		|	\
	 (1 << IEEE80211_RADIOTAP_LOCK_QUALITY)		|	\
	 (1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL)		|	\
	 0)

struct rtw_rx_radiotap_header {
	struct ieee80211_radiotap_header rr_ihdr;
	uint64_t	rr_tsft;
	uint8_t		rr_flags;
	uint8_t		rr_rate;
	uint16_t	rr_chan_freq;
	uint16_t	rr_chan_flags;
	uint16_t	rr_barker_lock;
	uint8_t		rr_antsignal;
} __packed;

#define RTW_TX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_FLAGS)	|	\
	 (1 << IEEE80211_RADIOTAP_RATE)		|	\
	 (1 << IEEE80211_RADIOTAP_CHANNEL)	|	\
	 0)

struct rtw_tx_radiotap_header {
	struct ieee80211_radiotap_header rt_ihdr;
	uint8_t		rt_flags;
	uint8_t		rt_rate;
	uint16_t	rt_chan_freq;
	uint16_t	rt_chan_flags;
} __packed;

enum rtw_attach_state {FINISHED, FINISH_DESCMAP_LOAD, FINISH_DESCMAP_CREATE,
	FINISH_DESC_MAP, FINISH_DESC_ALLOC, FINISH_RXMAPS_CREATE,
	FINISH_TXMAPS_CREATE, FINISH_RESET, FINISH_READ_SROM, FINISH_PARSE_SROM,
	FINISH_RF_ATTACH, FINISH_ID_STA, FINISH_TXDESCBLK_SETUP,
	FINISH_TXCTLBLK_SETUP, DETACHED};

struct rtw_hooks {
	void			*rh_shutdown;	/* shutdown hook */
	void			*rh_power;	/* power management hook */
};

struct rtw_mtbl {
	int			(*mt_newstate)(struct ieee80211com *,
					       enum ieee80211_state, int);
	void			(*mt_recv_mgmt)(struct ieee80211com *,
						struct mbuf *,
						struct ieee80211_node *,
						int, int, uint32_t);
	struct ieee80211_node	*(*mt_node_alloc)(struct ieee80211_node_table*);
	void			(*mt_node_free)(struct ieee80211_node *);
};

enum rtw_pwrstate { RTW_OFF = 0, RTW_SLEEP, RTW_ON };

typedef void (*rtw_continuous_tx_cb_t)(void *arg, int);

struct rtw_phy {
	struct rtw_rf	*p_rf;
	struct rtw_regs	*p_regs;
};

struct rtw_bbpset {
	u_int	bb_antatten;
	u_int	bb_chestlim;
	u_int	bb_chsqlim;
	u_int	bb_ifagcdet;
	u_int	bb_ifagcini;
	u_int	bb_ifagclimit;
	u_int	bb_lnadet;
	u_int	bb_sys1;
	u_int	bb_sys2;
	u_int	bb_sys3;
	u_int	bb_trl;
	u_int	bb_txagc;
};

struct rtw_rf {
	void			(*rf_destroy)(struct rtw_rf *);
	/* args: frequency, txpower, power state */
	int			(*rf_init)(struct rtw_rf *, u_int, uint8_t,
					   enum rtw_pwrstate);
	/* arg: power state */
	int			(*rf_pwrstate)(struct rtw_rf *,
					       enum rtw_pwrstate);
	/* arg: frequency */
	int			(*rf_tune)(struct rtw_rf *, u_int);
	/* arg: txpower */
	int			(*rf_txpower)(struct rtw_rf *, uint8_t);
	rtw_continuous_tx_cb_t	rf_continuous_tx_cb;
	void			*rf_continuous_tx_arg;
	struct rtw_bbpset	rf_bbpset;
};

static __inline void
rtw_rf_destroy(struct rtw_rf *rf)
{
	rf->rf_destroy(rf);
}

static __inline int
rtw_rf_init(struct rtw_rf *rf, u_int freq, uint8_t opaque_txpower,
	    enum rtw_pwrstate power)
{
	return rf->rf_init(rf, freq, opaque_txpower, power);
}

static __inline int
rtw_rf_pwrstate(struct rtw_rf *rf, enum rtw_pwrstate power)
{
	return rf->rf_pwrstate(rf, power);
}

static __inline int
rtw_rf_tune(struct rtw_rf *rf, u_int freq)
{
	return rf->rf_tune(rf, freq);
}

static __inline int
rtw_rf_txpower(struct rtw_rf *rf, uint8_t opaque_txpower)
{
	return rf->rf_txpower(rf, opaque_txpower);
}

typedef int (*rtw_rf_write_t)(struct rtw_regs *, enum rtw_rfchipid, u_int,
			      uint32_t);

struct rtw_rfbus {
	struct rtw_regs		*b_regs;
	rtw_rf_write_t		b_write;
};

static __inline int
rtw_rfbus_write(struct rtw_rfbus *bus, enum rtw_rfchipid rfchipid, u_int addr,
		uint32_t val)
{
	return bus->b_write(bus->b_regs, rfchipid, addr, val);
}

struct rtw_max2820 {
	struct rtw_rf		mx_rf;
	struct rtw_rfbus	mx_bus;
	int			mx_is_a;	/* 1: MAX2820A/MAX2821A */
};

struct rtw_grf5101 {
	struct rtw_rf		gr_rf;
	struct rtw_rfbus	gr_bus;
};

struct rtw_sa2400 {
	struct rtw_rf		sa_rf;
	struct rtw_rfbus	sa_bus;
	int			sa_digphy;	/* 1: digital PHY */
};

union rtw_keys {
	uint8_t		rk_keys[4][16];
	uint32_t	rk_words[16];
};

#define	RTW_LED_SLOW_TICKS	MAX(1, hz/2)
#define	RTW_LED_FAST_TICKS	MAX(1, hz/10)

struct rtw_led_state {
#define	RTW_LED0	0x1
#define	RTW_LED1	0x2
	uint8_t		ls_slowblink:2;
	uint8_t		ls_actblink:2;
	uint8_t		ls_default:2;
	uint8_t		ls_state;
	uint8_t		ls_event;
#define	RTW_LED_S_RX	0x1
#define	RTW_LED_S_TX	0x2
#define	RTW_LED_S_SLOW	0x4
	struct callout	ls_slow_ch;
	struct callout	ls_fast_ch;
};

struct rtw_softc {
	struct ieee80211com	sc_ic;
	struct rtw_regs		sc_regs;
	uint32_t		sc_flags;

	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_irq_handle;

	enum rtw_rfchipid	sc_rfchipid;
	enum rtw_locale		sc_locale;
	uint8_t			sc_phydelay;

	/* s/w Tx/Rx descriptors */
	bus_dma_tag_t		sc_txsoft_dmat;
	struct rtw_txsoft_blk	sc_txsoft_blk[RTW_NTXPRI];
	struct rtw_txdesc_blk	sc_txdesc_blk[RTW_NTXPRI];

	int			sc_rxsoft_free;
	bus_dmamap_t		sc_rxsoft_dmamap;
	bus_dma_tag_t		sc_rxsoft_dmat;
	struct rtw_rxsoft	sc_rxsoft[RTW_RXQLEN];
	struct rtw_rxdesc_blk	sc_rxdesc_blk;

	struct rtw_srom		sc_srom;

	enum rtw_pwrstate	sc_pwrstate;

	void			(*sc_pwrstate_cb)
				(struct rtw_regs *, enum rtw_pwrstate,
				 int, int);

	int			(*sc_getrssi)(uint8_t, uint8_t);

	struct rtw_rf		*sc_rf;

	uint16_t		sc_inten;

	/* interrupt acknowledge hook */
	void			(*sc_intr_ack)(struct rtw_regs *);

	void			(*sc_power)(struct rtw_softc *, int);
	struct rtw_mtbl		sc_mtbl;
	struct rtw_hooks	sc_hooks;

	struct bpf_if		*sc_radiobpf;

	struct callout		sc_scan_ch;
	u_int			sc_cur_chan;

	uint32_t		sc_tsfth;	/* most significant TSFT bits */
	uint32_t		sc_rcr;		/* RTW_RCR */
	uint8_t			sc_csthr;	/* carrier-sense threshold */

	int			sc_do_tick;	/* indicate 1s ticks */
	struct timeval		sc_tick0;	/* first tick */

	uint8_t			sc_rev;		/* PCI/Cardbus revision */

	uint32_t		sc_anaparm;	/* register RTW_ANAPARM */

	union {
		struct rtw_rx_radiotap_header tap;
		uint8_t		pad[64];
	} sc_rxtapu;
	union {
		struct rtw_tx_radiotap_header tap;
		uint8_t		pad[64];
	} sc_txtapu;
	union rtw_keys		sc_keys;
	struct ifqueue		sc_beaconq;
	struct rtw_led_state	sc_led_state;
	int			sc_hwverid;
	struct ieee80211_onoe_param sc_onoe_param;
};

#define	sc_if		sc_ic.ic_if
#define sc_rxtap	sc_rxtapu.tap
#define sc_txtap	sc_txtapu.tap

extern devclass_t	rtw_devclass;

#if 0
void		rtw_txdac_enable(struct rtw_softc *, int);
void		rtw_anaparm_enable(struct rtw_regs *, int);
void		rtw_config0123_enable(struct rtw_regs *, int);
void		rtw_continuous_tx_enable(struct rtw_softc *, int);
void		rtw_set_access(struct rtw_regs *, enum rtw_access);
#endif

int		rtw_attach(device_t);
int		rtw_detach(device_t);
void		rtw_stop(struct rtw_softc *, int);
#ifdef RTW_DEBUG
const char 	*rtw_pwrstate_string(enum rtw_pwrstate);
#endif

#endif /* _DEV_IC_RTWVAR_H_ */
