/*
 * Copyright (c) 1988 Stephen Deering.
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Stephen Deering of Stanford University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)igmp.c	8.1 (Berkeley) 7/19/93
 * $FreeBSD: src/sys/netinet/igmp.c,v 1.29.2.2 2003/01/23 21:06:44 sam Exp $
 */

/*
 * Internet Group Management Protocol (IGMP) routines.
 *
 * Written by Steve Deering, Stanford, May 1988.
 * Modified by Rosen Sharma, Stanford, Aug 1994.
 * Modified by Bill Fenner, Xerox PARC, Feb 1995.
 * Modified to fully comply to IGMPv2 by Bill Fenner, Oct 1995.
 *
 * MULTICAST Revision: 3.5.1.4
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/protosw.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <sys/in_cksum.h>
#include <sys/thread2.h>

#include <machine/stdarg.h>

#include <net/if.h>
#include <net/route.h>
#include <net/netmsg2.h>
#include <net/netisr2.h>

#include <netinet/in.h>
#include <netinet/in_var.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/ip_var.h>
#include <netinet/igmp.h>
#include <netinet/igmp_var.h>

#define IGMP_FASTTIMO		(hz / PR_FASTHZ)
#define IGMP_SLOWTIMO		(hz / PR_SLOWHZ)

static MALLOC_DEFINE(M_IGMP, "igmp", "igmp state");

static struct router_info *
		find_rti (struct ifnet *ifp);

static struct igmpstat igmpstat;

SYSCTL_STRUCT(_net_inet_igmp, IGMPCTL_STATS, stats, CTLFLAG_RW,
	&igmpstat, igmpstat, "IGMP statistics");

static int igmp_timers_are_running;
static u_long igmp_all_hosts_group;
static u_long igmp_all_rtrs_group;
static struct mbuf *router_alert;
static struct router_info *Head;

static void igmp_sendpkt (struct in_multi *, int, unsigned long);
static void igmp_fasttimo_dispatch(netmsg_t);
static void igmp_fasttimo(void *);
static void igmp_start_fasttimo(void);
static void igmp_slowtimo_dispatch(netmsg_t);
static void igmp_slowtimo(void *);
static void igmp_start_slowtimo(void);

static struct netmsg_base igmp_slowtimo_netmsg;
static struct periodic_call igmp_slowtimo_ch;
static struct netmsg_base igmp_fasttimo_netmsg;
static struct periodic_call igmp_fasttimo_ch;

void
igmp_init(void)
{
	struct ipoption *ra;
	int cpu;

	/*
	 * To avoid byte-swapping the same value over and over again.
	 */
	igmp_all_hosts_group = htonl(INADDR_ALLHOSTS_GROUP);
	igmp_all_rtrs_group = htonl(INADDR_ALLRTRS_GROUP);

	igmp_timers_are_running = 0;

	/*
	 * Construct a Router Alert option to use in outgoing packets
	 */
	MGET(router_alert, M_NOWAIT, MT_DATA);
	ra = mtod(router_alert, struct ipoption *);
	ra->ipopt_dst.s_addr = 0;
	ra->ipopt_list[0] = IPOPT_RA;	/* Router Alert Option */
	ra->ipopt_list[1] = 0x04;	/* 4 bytes long */
	ra->ipopt_list[2] = 0x00;
	ra->ipopt_list[3] = 0x00;
	router_alert->m_len = sizeof(ra->ipopt_dst) + ra->ipopt_list[1];

	Head = NULL;

	callout_init_periodic(&igmp_slowtimo_ch);
	netmsg_init(&igmp_slowtimo_netmsg, NULL, &netisr_adone_rport,
	    MSGF_PRIORITY, igmp_slowtimo_dispatch);

	callout_init_periodic(&igmp_fasttimo_ch);
	netmsg_init(&igmp_fasttimo_netmsg, NULL, &netisr_adone_rport,
	    MSGF_PRIORITY, igmp_fasttimo_dispatch);

	cpu = mycpuid;
	lwkt_migratecpu(0);
	callout_start_periodic(&igmp_slowtimo_ch, IGMP_SLOWTIMO,
	    igmp_slowtimo, NULL);
	callout_start_periodic(&igmp_fasttimo_ch, IGMP_FASTTIMO,
	    igmp_fasttimo, NULL);
	lwkt_migratecpu(cpu);
}

static struct router_info *
find_rti(struct ifnet *ifp)
{
	struct router_info *rti = Head;

#ifdef IGMP_DEBUG
	kprintf("[igmp.c, _find_rti] --> entering \n");
#endif
	while (rti) {
		if (rti->rti_ifp == ifp) {
#ifdef IGMP_DEBUG
			kprintf("[igmp.c, _find_rti] --> found old entry \n");
#endif
			return rti;
		}
		rti = rti->rti_next;
	}
	rti = kmalloc(sizeof *rti, M_IGMP, M_INTWAIT);
	rti->rti_ifp = ifp;
	rti->rti_type = IGMP_V2_ROUTER;
	rti->rti_time = 0;
	rti->rti_next = Head;
	Head = rti;
#ifdef IGMP_DEBUG
	kprintf("[igmp.c, _find_rti] --> created an entry \n");
#endif
	return rti;
}

static void
_igmp_start_slowtimo(void *arg)
{
	callout_start_periodic(&igmp_slowtimo_ch, IGMP_SLOWTIMO,
	    igmp_slowtimo, NULL);
}

static void
igmp_start_slowtimo(void)
{
	if (mycpuid == 0)
		_igmp_start_slowtimo(NULL);
	else
		lwkt_send_ipiq(globaldata_find(0), _igmp_start_slowtimo, NULL);
}

static void
_igmp_start_fasttimo(void *arg)
{
	callout_start_periodic(&igmp_fasttimo_ch, IGMP_FASTTIMO,
	    igmp_fasttimo, NULL);
}

static void
igmp_start_fasttimo(void)
{
	if (mycpuid == 0)
		_igmp_start_fasttimo(NULL);
	else
		lwkt_send_ipiq(globaldata_find(0), _igmp_start_fasttimo, NULL);
}

int
igmp_input(struct mbuf **mp, int *offp, int proto)
{
	struct mbuf *m = *mp;
	int iphlen;
	struct igmp *igmp;
	struct ip *ip;
	int igmplen;
	struct ifnet *ifp = m->m_pkthdr.rcvif;
	int minlen;
	struct in_multi *inm;
	struct in_ifaddr *ia;
	struct in_multistep step;
	struct router_info *rti;
	int timer; /** timer value in the igmp query header **/

	iphlen = *offp;
	*mp = NULL;

	++igmpstat.igps_rcv_total;

	ip = mtod(m, struct ip *);
	igmplen = ip->ip_len;

	/*
	 * Validate lengths
	 */
	if (igmplen < IGMP_MINLEN) {
		++igmpstat.igps_rcv_tooshort;
		m_freem(m);
		return(IPPROTO_DONE);
	}
	minlen = iphlen + IGMP_MINLEN;
	if ((m->m_flags & M_EXT || m->m_len < minlen) &&
	    (m = m_pullup(m, minlen)) == NULL) {
		++igmpstat.igps_rcv_tooshort;
		return(IPPROTO_DONE);
	}

	/*
	 * Validate checksum
	 */
	m->m_data += iphlen;
	m->m_len -= iphlen;
	igmp = mtod(m, struct igmp *);
	if (in_cksum(m, igmplen)) {
		++igmpstat.igps_rcv_badsum;
		m_freem(m);
		return(IPPROTO_DONE);
	}
	m->m_data -= iphlen;
	m->m_len += iphlen;

	ip = mtod(m, struct ip *);
	timer = igmp->igmp_code * PR_FASTHZ / IGMP_TIMER_SCALE;
	if (timer == 0)
		timer = 1;
	rti = find_rti(ifp);

	/*
	 * In the IGMPv2 specification, there are 3 states and a flag.
	 *
	 * In Non-Member state, we simply don't have a membership record.
	 * In Delaying Member state, our timer is running (inm->inm_timer)
	 * In Idle Member state, our timer is not running (inm->inm_timer==0)
	 *
	 * The flag is inm->inm_state, it is set to IGMP_OTHERMEMBER if
	 * we have heard a report from another member, or IGMP_IREPORTEDLAST
	 * if I sent the last report.
	 */
	switch (igmp->igmp_type) {

	case IGMP_MEMBERSHIP_QUERY:
		++igmpstat.igps_rcv_queries;

		if (ifp->if_flags & IFF_LOOPBACK)
			break;

		if (igmp->igmp_code == 0) {
			/*
			 * Old router.  Remember that the querier on this
			 * interface is old, and set the timer to the
			 * value in RFC 1112.
			 */

			rti->rti_type = IGMP_V1_ROUTER;
			rti->rti_time = 0;
			igmp_start_slowtimo();

			timer = IGMP_MAX_HOST_REPORT_DELAY * PR_FASTHZ;

			if (ip->ip_dst.s_addr != igmp_all_hosts_group ||
			    igmp->igmp_group.s_addr != 0) {
				++igmpstat.igps_rcv_badqueries;
				m_freem(m);
				return(IPPROTO_DONE);
			}
		} else {
			/*
			 * New router.  Simply do the new validity check.
			 */
			
			if (igmp->igmp_group.s_addr != 0 &&
			    !IN_MULTICAST(ntohl(igmp->igmp_group.s_addr))) {
				++igmpstat.igps_rcv_badqueries;
				m_freem(m);
				return(IPPROTO_DONE);
			}
		}

		/*
		 * - Start the timers in all of our membership records
		 *   that the query applies to for the interface on
		 *   which the query arrived excl. those that belong
		 *   to the "all-hosts" group (224.0.0.1).
		 * - Restart any timer that is already running but has
		 *   a value longer than the requested timeout.
		 * - Use the value specified in the query message as
		 *   the maximum timeout.
		 */
		IN_FIRST_MULTI(step, inm);
		while (inm != NULL) {
			if (inm->inm_ifp == ifp &&
			    inm->inm_addr.s_addr != igmp_all_hosts_group &&
			    (igmp->igmp_group.s_addr == 0 ||
			     igmp->igmp_group.s_addr == inm->inm_addr.s_addr)) {
				if (inm->inm_timer == 0 ||
				    inm->inm_timer > timer) {
					inm->inm_timer =
						IGMP_RANDOM_DELAY(timer);
					if (igmp_timers_are_running == 0)
						igmp_start_fasttimo();
					igmp_timers_are_running = 1;
				}
			}
			IN_NEXT_MULTI(step, inm);
		}

		break;

	case IGMP_V1_MEMBERSHIP_REPORT:
	case IGMP_V2_MEMBERSHIP_REPORT:
		/*
		 * For fast leave to work, we have to know that we are the
		 * last person to send a report for this group.  Reports
		 * can potentially get looped back if we are a multicast
		 * router, so discard reports sourced by me.
		 */
		ia = IFP_TO_IA(ifp);
		if (ia && ip->ip_src.s_addr == IA_SIN(ia)->sin_addr.s_addr)
			break;

		++igmpstat.igps_rcv_reports;

		if (ifp->if_flags & IFF_LOOPBACK)
			break;

		if (!IN_MULTICAST(ntohl(igmp->igmp_group.s_addr))) {
			++igmpstat.igps_rcv_badreports;
			m_freem(m);
			return(IPPROTO_DONE);
		}

		/*
		 * KLUDGE: if the IP source address of the report has an
		 * unspecified (i.e., zero) subnet number, as is allowed for
		 * a booting host, replace it with the correct subnet number
		 * so that a process-level multicast routing demon can
		 * determine which subnet it arrived from.  This is necessary
		 * to compensate for the lack of any way for a process to
		 * determine the arrival interface of an incoming packet.
		 */
		if ((ntohl(ip->ip_src.s_addr) & IN_CLASSA_NET) == 0)
			if (ia) ip->ip_src.s_addr = htonl(ia->ia_subnet);

		/*
		 * If we belong to the group being reported, stop
		 * our timer for that group.
		 */
		inm = IN_LOOKUP_MULTI(&igmp->igmp_group, ifp);

		if (inm != NULL) {
			inm->inm_timer = 0;
			++igmpstat.igps_rcv_ourreports;

			inm->inm_state = IGMP_OTHERMEMBER;
		}

		break;
	}

	/*
	 * Pass all valid IGMP packets up to any process(es) listening
	 * on a raw IGMP socket.
	 */
	*mp = m;
	rip_input(mp, offp, proto);
	return(IPPROTO_DONE);
}

void
igmp_joingroup(struct in_multi *inm)
{
	crit_enter();
	if (inm->inm_addr.s_addr == igmp_all_hosts_group
	    || inm->inm_ifp->if_flags & IFF_LOOPBACK) {
		inm->inm_timer = 0;
		inm->inm_state = IGMP_OTHERMEMBER;
	} else {
		inm->inm_rti = find_rti(inm->inm_ifp);
		igmp_sendpkt(inm, inm->inm_rti->rti_type, 0);
		inm->inm_timer = IGMP_RANDOM_DELAY(
					IGMP_MAX_HOST_REPORT_DELAY*PR_FASTHZ);
		inm->inm_state = IGMP_IREPORTEDLAST;
		if (igmp_timers_are_running == 0)
			igmp_start_fasttimo();
		igmp_timers_are_running = 1;
	}
	crit_exit();
}

void
igmp_leavegroup(struct in_multi *inm)
{
	if (inm->inm_state == IGMP_IREPORTEDLAST &&
	    inm->inm_addr.s_addr != igmp_all_hosts_group &&
	    !(inm->inm_ifp->if_flags & IFF_LOOPBACK) &&
	    inm->inm_rti->rti_type != IGMP_V1_ROUTER)
		igmp_sendpkt(inm, IGMP_V2_LEAVE_GROUP, igmp_all_rtrs_group);
}

static void
igmp_fasttimo(void *dummy __unused)
{
	struct netmsg_base *msg = &igmp_fasttimo_netmsg;

	KKASSERT(mycpuid == 0);

	crit_enter();
	if (msg->lmsg.ms_flags & MSGF_DONE)
		netisr_sendmsg_oncpu(msg);
	crit_exit();
}

static void
igmp_fasttimo_dispatch(netmsg_t nmsg)
{
	struct in_multi *inm;
	struct in_multistep step;

	ASSERT_NETISR0;

	crit_enter();
	netisr_replymsg(&nmsg->base, 0);	/* reply ASAP */
	crit_exit();

	/*
	 * Quick check to see if any work needs to be done, in order
	 * to minimize the overhead of fasttimo processing.
	 */

	if (!igmp_timers_are_running) {
		callout_stop_periodic(&igmp_fasttimo_ch);
//		goto done;
	}

	igmp_timers_are_running = 0;
	IN_FIRST_MULTI(step, inm);
	while (inm != NULL) {
		if (inm->inm_timer == 0) {
			/* do nothing */
		} else if (--inm->inm_timer == 0) {
			igmp_sendpkt(inm, inm->inm_rti->rti_type, 0);
			inm->inm_state = IGMP_IREPORTEDLAST;
		} else {
			igmp_timers_are_running = 1;
		}
		IN_NEXT_MULTI(step, inm);
	}
//done:
	;
//	callout_reset(&igmp_fasttimo_ch, IGMP_FASTTIMO, igmp_fasttimo, NULL);
}

static void
igmp_slowtimo(void *dummy __unused)
{
	struct netmsg_base *msg = &igmp_slowtimo_netmsg;

	KKASSERT(mycpuid == 0);

	crit_enter();
	if (msg->lmsg.ms_flags & MSGF_DONE)
		netisr_sendmsg_oncpu(msg);
	crit_exit();
}

static void
igmp_slowtimo_dispatch(netmsg_t nmsg)
{
	struct router_info *rti = Head;
	int any = 0;

	ASSERT_NETISR0;

	crit_enter();
	netisr_replymsg(&nmsg->base, 0);	/* reply ASAP */
	crit_exit();

#ifdef IGMP_DEBUG
	kprintf("[igmp.c,_slowtimo] -- > entering \n");
#endif
	while (rti) {
	    if (rti->rti_type == IGMP_V1_ROUTER) {
		any = 1;
		rti->rti_time++;
		if (rti->rti_time >= IGMP_AGE_THRESHOLD) {
			rti->rti_type = IGMP_V2_ROUTER;
		}
	    }
	    rti = rti->rti_next;
	}
	if (any == 0)
		callout_stop_periodic(&igmp_slowtimo_ch);
#ifdef IGMP_DEBUG	
	kprintf("[igmp.c,_slowtimo] -- > exiting \n");
#endif
//	callout_reset(&igmp_slowtimo_ch, IGMP_SLOWTIMO, igmp_slowtimo, NULL);
}

static struct route igmprt;

static void
igmp_sendpkt(struct in_multi *inm, int type, unsigned long addr)
{
	struct mbuf *m;
	struct igmp *igmp;
	struct ip *ip;
	struct ip_moptions imo;

	MGETHDR(m, M_NOWAIT, MT_HEADER);
	if (m == NULL)
		return;

	m->m_pkthdr.rcvif = loif;
	m->m_pkthdr.len = sizeof(struct ip) + IGMP_MINLEN;
	MH_ALIGN(m, IGMP_MINLEN + sizeof(struct ip));
	m->m_data += sizeof(struct ip);
	m->m_len = IGMP_MINLEN;
	igmp = mtod(m, struct igmp *);
	igmp->igmp_type   = type;
	igmp->igmp_code   = 0;
	igmp->igmp_group  = inm->inm_addr;
	igmp->igmp_cksum  = 0;
	igmp->igmp_cksum  = in_cksum(m, IGMP_MINLEN);

	m->m_data -= sizeof(struct ip);
	m->m_len += sizeof(struct ip);
	ip = mtod(m, struct ip *);
	ip->ip_tos = 0;
	ip->ip_len = sizeof(struct ip) + IGMP_MINLEN;
	ip->ip_off = 0;
	ip->ip_p = IPPROTO_IGMP;
	ip->ip_src.s_addr = INADDR_ANY;
	ip->ip_dst.s_addr = addr ? addr : igmp->igmp_group.s_addr;

	imo.imo_multicast_ifp = inm->inm_ifp;
	imo.imo_multicast_ttl = 1;
	imo.imo_multicast_vif = -1;
	/*
	 * Request loopback of the report if we are acting as a multicast
	 * router, so that the process-level routing demon can hear it.
	 */
	imo.imo_multicast_loop = (ip_mrouter != NULL);

	/*
	 * XXX
	 * Do we have to worry about reentrancy here?  Don't think so.
	 */
	ip_output(m, router_alert, &igmprt, 0, &imo, NULL);

	++igmpstat.igps_snd_reports;
}
