/*	$OpenBSD: if_nfevar.h,v 1.11 2006/02/19 13:57:02 damien Exp $	*/
/*	$DragonFly: src/sys/dev/netif/nfe/if_nfevar.h,v 1.8 2008/07/05 05:34:31 sephe Exp $	*/

/*
 * Copyright (c) 2005 Jonathan Gray <jsg@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#define NFE_IFQ_MAXLEN	64

struct nfe_tx_data {
	bus_dmamap_t	map;
	struct mbuf	*m;
};

struct nfe_tx_ring {
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	bus_addr_t		physaddr;
	struct nfe_desc32	*desc32;
	struct nfe_desc64	*desc64;

	bus_dma_tag_t		data_tag;
	struct nfe_tx_data	data[NFE_TX_RING_COUNT];
	int			queued;
	int			cur;
	int			next;
};

struct nfe_softc;

struct nfe_jbuf {
	struct nfe_softc	*sc;
	struct nfe_rx_ring	*ring;
	int			inuse;
	int			slot;
	caddr_t			buf;
	bus_addr_t		physaddr;
	SLIST_ENTRY(nfe_jbuf)	jnext;
};

struct nfe_rx_data {
	bus_dmamap_t	map;
	struct mbuf	*m;
};

struct nfe_rx_ring {
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	bus_addr_t		physaddr;
	struct nfe_desc32	*desc32;
	struct nfe_desc64	*desc64;

	bus_dma_tag_t		jtag;
	bus_dmamap_t		jmap;
	caddr_t			jpool;
	struct nfe_jbuf		*jbuf;
	SLIST_HEAD(, nfe_jbuf)	jfreelist;

	bus_dma_tag_t		data_tag;
	bus_dmamap_t		data_tmpmap;
	struct nfe_rx_data	*data;
	int			bufsz;
	int			cur;
	int			next;
};

struct nfe_softc {
	struct arpcom		arpcom;

	int			sc_mem_rid;
	struct resource		*sc_mem_res;
	bus_space_handle_t	sc_memh;
	bus_space_tag_t		sc_memt;

	int			sc_irq_rid;
	struct resource		*sc_irq_res;
	void			*sc_ih;

	device_t		sc_miibus;
	struct callout		sc_tick_ch;

	int			sc_if_flags;
	uint32_t		sc_caps;
#define NFE_JUMBO_SUP	0x01
#define NFE_40BIT_ADDR	0x02
#define NFE_HW_CSUM	0x04
#define NFE_HW_VLAN	0x08
#define NFE_NO_PWRCTL	0x20

	uint32_t		sc_flags;
#define NFE_F_USE_JUMBO	0x01

	uint32_t		rxtxctl_desc;
	uint32_t		rxtxctl;
	uint8_t			mii_phyaddr;

	struct nfe_tx_ring	txq;
	struct nfe_rx_ring	rxq;

	uint32_t		sc_irq_enable;
	int			sc_imtime;
	int			sc_rx_ring_count;
	int			sc_debug;
	struct sysctl_ctx_list	sc_sysctl_ctx;
	struct sysctl_oid	*sc_sysctl_tree;

	struct lwkt_serialize	sc_jbuf_serializer;
};

#define NFE_IRQ_ENABLE(sc)	\
	((sc)->sc_imtime < 0 ? NFE_IRQ_NOIMTIMER : NFE_IRQ_IMTIMER)
