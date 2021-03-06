/*	$NetBSD: mii_physubr.c,v 1.49 2006/03/29 07:05:24 thorpej Exp $	*/

/*-
 * Copyright (c) 1998, 1999, 2000, 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/mii/mii_physubr.c,v 1.2.2.1 2000/12/12 19:29:14 wpaul Exp $
 */

/*
 * Subroutines common to all PHYs.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/thread2.h>

#include <net/if.h>
#include <net/if_media.h>

#include "mii.h"
#include "miivar.h"

#include "miibus_if.h"

const struct mii_media	mii_media_table[MII_NMEDIA] = {
	[MII_MEDIA_NONE]	= { .mm_bmcr = BMCR_ISO,
				    .mm_anar = ANAR_CSMA,
				    .mm_gtcr = 0 },

	[MII_MEDIA_10_T]	= { .mm_bmcr = BMCR_S10,
				    .mm_anar = ANAR_CSMA | ANAR_10,
				    .mm_gtcr = 0 },

	[MII_MEDIA_10_T_FDX]	= { .mm_bmcr = BMCR_S10 | BMCR_FDX,
				    .mm_anar = ANAR_CSMA | ANAR_10_FD,
				    .mm_gtcr = 0 },

	[MII_MEDIA_100_T4]	= { .mm_bmcr = BMCR_S100,
				    .mm_anar = ANAR_CSMA | ANAR_T4,
				    .mm_gtcr = 0 },

	[MII_MEDIA_100_TX]	= { .mm_bmcr = BMCR_S100,
				    .mm_anar = ANAR_CSMA | ANAR_TX,
				    .mm_gtcr = 0 },

	[MII_MEDIA_100_TX_FDX]	= { .mm_bmcr = BMCR_S100 | BMCR_FDX,
				    .mm_anar = ANAR_CSMA | ANAR_TX_FD,
				    .mm_gtcr = 0 },

	[MII_MEDIA_1000_X]	= { .mm_bmcr = BMCR_S1000,
				    .mm_anar = ANAR_CSMA,
				    .mm_gtcr = 0 },

	[MII_MEDIA_1000_X_FDX]	= { .mm_bmcr = BMCR_S1000 | BMCR_FDX,
				    .mm_anar = ANAR_CSMA,
				    .mm_gtcr = 0 },

	[MII_MEDIA_1000_T]	= { .mm_bmcr = BMCR_S1000,
				    .mm_anar = ANAR_CSMA,
				    .mm_gtcr = GTCR_ADV_1000THDX },

	[MII_MEDIA_1000_T_FDX]	= { .mm_bmcr = BMCR_S1000,
				    .mm_anar = ANAR_CSMA,
				    .mm_gtcr = GTCR_ADV_1000TFDX }
};

void
mii_softc_init(struct mii_softc *mii, struct mii_attach_args *ma)
{
	mii->mii_phy = ma->mii_phyno;
	mii->mii_flags |= ma->mii_flags;
	mii->mii_model = MII_MODEL(ma->mii_id2);
	mii->mii_rev = MII_REV(ma->mii_id2);
	mii->mii_privtag = ma->mii_privtag;
	mii->mii_priv = ma->mii_priv;

	if (mii->mii_reset == NULL)
		mii->mii_reset = mii_phy_reset;
	if (mii->mii_anegticks == 0)
		mii->mii_anegticks = MII_ANEGTICKS;
}

int
mii_phy_auto(struct mii_softc *sc, int waitfor)
{
	uint16_t anar;

	/*
	 * Check for 1000BASE-X.  Autonegotiation is a bit
	 * different on such devices.
	 */
	if (sc->mii_flags & MIIF_IS_1000X) {
		anar = 0;
		if (sc->mii_extcapabilities & EXTSR_1000XFDX)
			anar |= ANAR_X_FD;
		if (sc->mii_extcapabilities & EXTSR_1000XHDX)
			anar |= ANAR_X_HD;

		if (sc->mii_flags & MIIF_DOPAUSE) {
			/* XXX Asymmetric vs. symmetric? */
			anar |= ANLPAR_X_PAUSE_TOWARDS;
		}
		PHY_WRITE(sc, MII_ANAR, anar);
	} else {
		anar = BMSR_MEDIA_TO_ANAR(sc->mii_capabilities) |
		    ANAR_CSMA;
		if (sc->mii_flags & MIIF_DOPAUSE) {
			anar |= ANAR_FC;
			/* XXX Only 1000BASE-T has PAUSE_ASYM? */
			if ((sc->mii_flags & MIIF_HAVE_GTCR) &&
			    (sc->mii_extcapabilities &
			     (EXTSR_1000THDX|EXTSR_1000TFDX)))
				anar |= ANAR_X_PAUSE_ASYM;
		}
		PHY_WRITE(sc, MII_ANAR, anar);
		if (sc->mii_flags & MIIF_HAVE_GTCR) {
			uint16_t gtcr = 0;

			if (sc->mii_extcapabilities & EXTSR_1000TFDX)
				gtcr |= GTCR_ADV_1000TFDX;
			if (sc->mii_extcapabilities & EXTSR_1000THDX)
				gtcr |= GTCR_ADV_1000THDX;

			PHY_WRITE(sc, MII_100T2CR, gtcr);
		}
	}
	PHY_WRITE(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);

	if (waitfor) {
		int i;

		/* Wait 500ms for it to complete. */
		for (i = 0; i < 500; i++) {
			if (PHY_READ(sc, MII_BMSR) & BMSR_ACOMP)
				return (0);
			DELAY(1000);
		}
		return (EIO);
	}
	return (EJUSTRETURN);
}

void
mii_phy_reset(struct mii_softc *sc)
{
	int reg, i;

	if (sc->mii_flags & MIIF_NOISOLATE)
		reg = BMCR_RESET;
	else
		reg = BMCR_RESET | BMCR_ISO;
	PHY_WRITE(sc, MII_BMCR, reg);

	/*
	 * It is best to allow a little time for the reset to settle
	 * in before we start polling the BMCR again.  Notably, the
	 * DP83840A manual states that there should be a 500us delay
	 * between asserting software reset and attempting MII serial
	 * operations.  Also, a DP83815 can get into a bad state on
	 * cable removal and reinsertion if we do not delay here.
	 */
	DELAY(500);

	/* Wait 100ms for it to complete. */
	for (i = 0; i < 100; i++) {
		reg = PHY_READ(sc, MII_BMCR); 
		if ((reg & BMCR_RESET) == 0)
			break;
		DELAY(1000);
	}

	if (sc->mii_inst != 0 && ((sc->mii_flags & MIIF_NOISOLATE) == 0))
		PHY_WRITE(sc, MII_BMCR, reg | BMCR_ISO);
}

/*
 * Initialize generic PHY media based on BMSR, called when a PHY is
 * attached.  We expect to be set up to print a comma-separated list
 * of media names.  Does not print a newline.
 */
void
mii_phy_add_media(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	const char *sep = "";
	int fdx = 0;

#define	ADD(m, c)	ifmedia_add(&mii->mii_media, (m), (c), NULL)
#define	PRINT(s)	kprintf("%s%s", sep, s); sep = ", "

	if ((sc->mii_flags & MIIF_NOISOLATE) == 0)
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_NONE, 0, sc->mii_inst),
		    MII_MEDIA_NONE);

	/*
	 * There are different interpretations for the bits in
	 * HomePNA PHYs.  And there is really only one media type
	 * that is supported.
	 */
	if (sc->mii_flags & MIIF_IS_HPNA) {
		if (sc->mii_capabilities & BMSR_10THDX) {
			ADD(IFM_MAKEWORD(IFM_ETHER, IFM_HPNA_1, 0,
					 sc->mii_inst),
			    MII_MEDIA_10_T);
			PRINT("HomePNA1");
		}
		return;
	}

	if (sc->mii_capabilities & BMSR_10THDX) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_10_T, 0, sc->mii_inst),
		    MII_MEDIA_10_T);
		PRINT("10baseT");
	}
	if (sc->mii_capabilities & BMSR_10TFDX) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_10_T, IFM_FDX, sc->mii_inst),
		    MII_MEDIA_10_T_FDX);
		PRINT("10baseT-FDX");
		fdx = 1;
	}
	if (sc->mii_capabilities & BMSR_100TXHDX) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_100_TX, 0, sc->mii_inst),
		    MII_MEDIA_100_TX);
		PRINT("100baseTX");
	}
	if (sc->mii_capabilities & BMSR_100TXFDX) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_100_TX, IFM_FDX, sc->mii_inst),
		    MII_MEDIA_100_TX_FDX);
		PRINT("100baseTX-FDX");
		fdx = 1;
	}
	if (sc->mii_capabilities & BMSR_100T4) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_100_T4, 0, sc->mii_inst),
		    MII_MEDIA_100_T4);
		PRINT("100baseT4");
	}

	if (sc->mii_extcapabilities & EXTSR_MEDIAMASK) {
		/*
		 * XXX Right now only handle 1000SX and 1000TX.  Need
		 * XXX to handle 1000LX and 1000CX some how.
		 *
		 * Note since it can take 5 seconds to auto-negotiate
		 * a gigabit link, we make anegticks 10 seconds for
		 * all the gigabit media types.
		 */
		if (sc->mii_extcapabilities & EXTSR_1000XHDX) {
			sc->mii_anegticks = MII_ANEGTICKS_GIGE;
			sc->mii_flags |= MIIF_IS_1000X;
			ADD(IFM_MAKEWORD(IFM_ETHER, IFM_1000_SX, 0,
			    sc->mii_inst), MII_MEDIA_1000_X);
			PRINT("1000baseSX");
		}
		if (sc->mii_extcapabilities & EXTSR_1000XFDX) {
			sc->mii_anegticks = MII_ANEGTICKS_GIGE;
			sc->mii_flags |= MIIF_IS_1000X;
			ADD(IFM_MAKEWORD(IFM_ETHER, IFM_1000_SX, IFM_FDX,
			    sc->mii_inst), MII_MEDIA_1000_X_FDX);
			PRINT("1000baseSX-FDX");
			fdx = 1;
		}

		/*
		 * 1000baseT media needs to be able to manipulate
		 * master/slave mode.  We set IFM_ETH_MASTER in
		 * the "don't care mask" and filter it out when
		 * the media is set.
		 *
		 * All 1000baseT PHYs have a 1000baseT control register.
		 */
		if (sc->mii_extcapabilities & EXTSR_1000THDX) {
			sc->mii_anegticks = MII_ANEGTICKS_GIGE;
			sc->mii_flags |= MIIF_HAVE_GTCR;
			mii->mii_media.ifm_mask |= IFM_ETH_MASTER;
			ADD(IFM_MAKEWORD(IFM_ETHER, IFM_1000_T, 0,
			    sc->mii_inst), MII_MEDIA_1000_T);
			PRINT("1000baseT");
		}
		if (sc->mii_extcapabilities & EXTSR_1000TFDX) {
			sc->mii_anegticks = MII_ANEGTICKS_GIGE;
			sc->mii_flags |= MIIF_HAVE_GTCR;
			mii->mii_media.ifm_mask |= IFM_ETH_MASTER;
			ADD(IFM_MAKEWORD(IFM_ETHER, IFM_1000_T, IFM_FDX,
			    sc->mii_inst), MII_MEDIA_1000_T_FDX);
			PRINT("1000baseT-FDX");
			fdx = 1;
		}
	}

	if (sc->mii_capabilities & BMSR_ANEG) {
		ADD(IFM_MAKEWORD(IFM_ETHER, IFM_AUTO, 0, sc->mii_inst),
		    MII_NMEDIA);	/* intentionally invalid index */
		PRINT("auto");
	}
#undef ADD
#undef PRINT
	if (fdx != 0 && (sc->mii_flags & MIIF_DOPAUSE))
		mii->mii_media.ifm_mask |= IFM_ETH_FMASK;
}

void
mii_phy_set_media(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	struct ifmedia_entry *ife = mii->mii_media.ifm_cur;
	int bmcr, anar, gtcr;

	if (IFM_SUBTYPE(ife->ifm_media) == IFM_AUTO) {
		/*
		 * Force renegotiation if MIIF_DOPAUSE.
		 *
		 * XXX This is only necessary because many NICs don't
		 * XXX advertise PAUSE capabilities at boot time.  Maybe
		 * XXX we should force this only once?
		 */
		if ((PHY_READ(sc, MII_BMCR) & BMCR_AUTOEN) == 0 ||
		    (sc->mii_flags & (MIIF_FORCEANEG | MIIF_DOPAUSE)))
			mii_phy_auto(sc, 1);
		return;
	}

	/*
	 * Table index is stored in the media entry.
	 */

	KASSERT(ife->ifm_data >= 0 && ife->ifm_data < MII_NMEDIA,
		("bogus ife->ifm_data (%d)", ife->ifm_data));

	anar = mii_media_table[ife->ifm_data].mm_anar;
	bmcr = mii_media_table[ife->ifm_data].mm_bmcr;
	gtcr = mii_media_table[ife->ifm_data].mm_gtcr;

	if (mii->mii_media.ifm_media & IFM_ETH_MASTER) {
		switch (IFM_SUBTYPE(ife->ifm_media)) {
		case IFM_1000_T:
			gtcr |= GTCR_MAN_MS|GTCR_ADV_MS;
			break;

		default:
			panic("mii_phy_setmedia: MASTER on wrong media");
		}
	}

	if (mii->mii_media.ifm_media & IFM_FLOW) {
		if (sc->mii_flags & MIIF_IS_1000X) {
			anar |= ANAR_X_PAUSE_SYM | ANAR_X_PAUSE_ASYM;
		} else {
			anar |= ANAR_FC;
			/* XXX Only 1000BASE-T has PAUSE_ASYM? */
			if ((sc->mii_flags & MIIF_HAVE_GTCR) &&
			    (sc->mii_extcapabilities &
			     (EXTSR_1000THDX | EXTSR_1000TFDX)))
				anar |= ANAR_X_PAUSE_ASYM;
		}
	}

	if (ife->ifm_media & IFM_LOOP)
		bmcr |= BMCR_LOOP;

	PHY_WRITE(sc, MII_ANAR, anar);
	PHY_WRITE(sc, MII_BMCR, bmcr);
	if (sc->mii_flags & MIIF_HAVE_GTCR)
		PHY_WRITE(sc, MII_100T2CR, gtcr);
}

int
mii_phy_tick(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	struct ifmedia_entry *ife = mii->mii_media.ifm_cur;
	int reg;

	/* Just bail now if the interface is down. */
	if ((mii->mii_ifp->if_flags & IFF_UP) == 0)
		return (EJUSTRETURN);

	/*
	 * If we're not doing autonegotiation, we don't need to do
	 * any extra work here.  However, we need to check the link
	 * status so we can generate an announcement if the status
	 * changes.
	 */
	if (IFM_SUBTYPE(ife->ifm_media) != IFM_AUTO)
		return (0);

	/* Read the status register twice; BMSR_LINK is latch-low. */
	reg = PHY_READ(sc, MII_BMSR) | PHY_READ(sc, MII_BMSR);
	if (reg & BMSR_LINK) {
		/*
		 * See above.
		 */

		/* Reset autonegotiation timer. */
		sc->mii_ticks = 0;
		return (0);
	}

	/*
	 * Only retry autonegotiation every N seconds.
	 */
	KKASSERT(sc->mii_anegticks > 0);
	if (++sc->mii_ticks <= sc->mii_anegticks)
		return (EJUSTRETURN);

	sc->mii_ticks = 0;
	sc->mii_reset(sc);	/* Reset PHY */
	mii_phy_auto(sc, 0);	/* Ignore EJUSTRETURN */
	return (0);
}

static int
mii_phy_statusmsg(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	struct ifnet *ifp = mii->mii_ifp;
	int baudrate, link_state, announce = 0;

	if (mii->mii_media_status & IFM_AVALID) {
		if (mii->mii_media_status & IFM_ACTIVE)
			link_state = LINK_STATE_UP;
		else
			link_state = LINK_STATE_DOWN;
	} else
		link_state = LINK_STATE_UNKNOWN;

	baudrate = ifmedia_baudrate(mii->mii_media_active);

	if (link_state != ifp->if_link_state) {
		ifp->if_link_state = link_state;
		/*
		 * XXX Right here we'd like to notify protocols
		 * XXX that the link status has changed, so that
		 * XXX e.g. Duplicate Address Detection can restart.
		 */
		announce = 1;
	}

	if (baudrate != ifp->if_baudrate) {
		ifp->if_baudrate = baudrate;
		announce = 1;
	}

	return (announce);
}

void
mii_phy_update(struct mii_softc *sc, int cmd)
{
	struct mii_data *mii = sc->mii_pdata;
	struct ifnet *ifp = mii->mii_ifp;
	int announce;

	if (sc->mii_media_active != mii->mii_media_active ||
	    sc->mii_media_status != mii->mii_media_status ||
	    cmd == MII_MEDIACHG) {
		announce = mii_phy_statusmsg(sc);
		MIIBUS_STATCHG(sc->mii_dev);
		sc->mii_media_active = mii->mii_media_active;
		sc->mii_media_status = mii->mii_media_status;

		if (announce) {
			crit_enter();
			if_link_state_change(ifp);
			crit_exit();
		}
	}
}

/*
 * Return the flow control status flag from MII_ANAR & MII_ANLPAR.
 */
int
mii_phy_flowstatus(struct mii_softc *sc)
{
	int anar, anlpar;

	if ((sc->mii_flags & MIIF_DOPAUSE) == 0)
		return (0);

	anar = PHY_READ(sc, MII_ANAR);
	anlpar = PHY_READ(sc, MII_ANLPAR);

	if ((anar & ANAR_X_PAUSE_SYM) & (anlpar & ANLPAR_X_PAUSE_SYM))
		return (IFM_FLOW | IFM_ETH_TXPAUSE | IFM_ETH_RXPAUSE);

	if ((anar & ANAR_X_PAUSE_SYM) == 0) {
		if ((anar & ANAR_X_PAUSE_ASYM) &&
		    ((anlpar & ANLPAR_X_PAUSE_TOWARDS) ==
		     ANLPAR_X_PAUSE_TOWARDS))
			return (IFM_FLOW | IFM_ETH_TXPAUSE);
		else
			return (0);
	}

	if ((anar & ANAR_X_PAUSE_ASYM) == 0) {
		if (anlpar & ANLPAR_X_PAUSE_SYM)
			return (IFM_FLOW | IFM_ETH_TXPAUSE | IFM_ETH_RXPAUSE);
		else
			return (0);
	}

	switch (anlpar & ANLPAR_X_PAUSE_TOWARDS) {
	case ANLPAR_X_PAUSE_NONE:
		return (0);

	case ANLPAR_X_PAUSE_ASYM:
		return (IFM_FLOW | IFM_ETH_RXPAUSE);

	default:
		return (IFM_FLOW | IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE);
	}
	/* NOTREACHED */
}

const struct mii_phydesc *
mii_phy_match(const struct mii_attach_args *ma, const struct mii_phydesc *mpd)
{
	for (; mpd->mpd_name != NULL; mpd++) {
		if (MII_OUI(ma->mii_id1, ma->mii_id2) == mpd->mpd_oui &&
		    MII_MODEL(ma->mii_id2) == mpd->mpd_model)
			return (mpd);
	}
	return (NULL);
}
