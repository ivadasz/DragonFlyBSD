/*-
 * Copyright (c) 2001-2002 Luigi Rizzo
 *
 * Supported by: the Xorp Project (www.xorp.org)
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/kern/kern_poll.c,v 1.2.2.4 2002/06/27 23:26:33 luigi Exp $
 */

#include "opt_ifpoll.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/serialize.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/microtime_pcpu.h>

#include <sys/thread2.h>
#include <sys/msgport2.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_poll.h>
#include <net/netmsg2.h>
#include <net/netisr2.h>

/*
 * Polling support for network device drivers.
 *
 * Drivers which support this feature try to register one status polling
 * handler and several TX/RX polling handlers with the polling code.
 * If interface's if_npoll is called with non-NULL second argument, then
 * a register operation is requested, else a deregister operation is
 * requested.  If the requested operation is "register", driver should
 * setup the ifpoll_info passed in accoding its own needs:
 *   ifpoll_info.ifpi_status.status_func == NULL
 *     No status polling handler will be installed on CPU(0)
 *   ifpoll_info.ifpi_rx[n].poll_func == NULL
 *     No RX polling handler will be installed on CPU(n)
 *   ifpoll_info.ifpi_tx[n].poll_func == NULL
 *     No TX polling handler will be installed on CPU(n)
 *
 * Serializer field of ifpoll_info.ifpi_status and ifpoll_info.ifpi_tx[n]
 * must _not_ be NULL.  The serializer will be held before the status_func
 * and poll_func being called.  Serializer field of ifpoll_info.ifpi_rx[n]
 * can be NULL, but the interface's if_flags must have IFF_IDIRECT set,
 * which indicates that the network processing of the input packets is
 * running directly instead of being redispatched.
 *
 * RX is polled at the specified polling frequency (net.ifpoll.X.pollhz).
 * TX and status polling could be done at lower frequency than RX frequency
 * (net.ifpoll.0.status_frac and net.ifpoll.X.tx_frac).  To avoid systimer
 * staggering at high frequency, RX systimer gives TX and status polling a
 * piggyback (XXX).
 *
 * All of the registered polling handlers are called only if the interface
 * is marked as IFF_UP, IFF_RUNNING and IFF_NPOLLING.  However, the
 * interface's register and deregister function (ifnet.if_npoll) will be
 * called even if interface is not marked with IFF_RUNNING or IFF_UP.
 *
 * If registration is successful, the driver must disable interrupts,
 * and further I/O is performed through the TX/RX polling handler, which
 * are invoked (at least once per clock tick) with 3 arguments: the "arg"
 * passed at register time, a struct ifnet pointer, and a "count" limit.
 * The registered serializer will be held before calling the related
 * polling handler.
 *
 * The count limit specifies how much work the handler can do during the
 * call -- typically this is the number of packets to be received, or
 * transmitted, etc. (drivers are free to interpret this number, as long
 * as the max time spent in the function grows roughly linearly with the
 * count).
 *
 * A second variable controls the sharing of CPU between polling/kernel
 * network processing, and other activities (typically userlevel tasks):
 * net.ifpoll.X.{rx,tx}.user_frac (between 0 and 100, default 50) sets the
 * share of CPU allocated to user tasks.  CPU is allocated proportionally
 * to the shares, by dynamically adjusting the "count" (poll_burst).
 *
 * Other parameters can should be left to their default values.
 * The following constraints hold
 *
 *	1 <= poll_burst <= poll_burst_max
 *	1 <= poll_each_burst <= poll_burst_max
 *	MIN_POLL_BURST_MAX <= poll_burst_max <= MAX_POLL_BURST_MAX
 */

#define IFPOLL_LIST_LEN		128
#define IFPOLL_FREQ_MAX		30000

#define MIN_IOPOLL_BURST_MAX	10
#define MAX_IOPOLL_BURST_MAX	5000
#define IOPOLL_BURST_MAX	250	/* good for 1000Mbit net and HZ=6000 */

#define IOPOLL_EACH_BURST	50
#define IOPOLL_USER_FRAC	50

#define IFPOLL_FREQ_DEFAULT	6000

#define IFPOLL_TXFRAC_DEFAULT	1	/* 1/1 of the pollhz */
#define IFPOLL_STFRAC_DEFAULT	120	/* 1/120 of the pollhz */

#define IFPOLL_RX		0x1
#define IFPOLL_TX		0x2

struct iopoll_rec {
	struct lwkt_serialize	*serializer;
	struct ifnet		*ifp;
	void			*arg;
	ifpoll_iofn_t		poll_func;
};

struct iopoll_ctx {
	union microtime_pcpu	prev_t;
	u_long			short_ticks;		/* statistics */
	u_long			lost_polls;		/* statistics */
	u_long			suspect;		/* statistics */
	u_long			stalled;		/* statistics */
	uint32_t		pending_polls;		/* state */

	struct netmsg_base	poll_netmsg;
	struct netmsg_base	poll_more_netmsg;

	int			poll_cpuid;
	int			pollhz;
	uint32_t		phase;			/* state */
	int			residual_burst;		/* state */
	uint32_t		poll_each_burst;	/* tunable */
	union microtime_pcpu	poll_start_t;		/* state */

	uint32_t		poll_burst;		/* state */
	uint32_t		poll_burst_max;		/* tunable */
	uint32_t		user_frac;		/* tunable */
	uint32_t		kern_frac;		/* state */

	uint32_t		poll_handlers; /* next free entry in pr[]. */
	struct iopoll_rec	pr[IFPOLL_LIST_LEN];

	struct sysctl_ctx_list	poll_sysctl_ctx;
	struct sysctl_oid	*poll_sysctl_tree;
};

struct poll_comm {
	struct systimer		pollclock;
	int			poll_cpuid;

	int			stfrac_count;		/* state */
	int			poll_stfrac;		/* tunable */

	int			txfrac_count;		/* state */
	int			poll_txfrac;		/* tunable */

	int			pollhz;			/* tunable */

	struct sysctl_ctx_list	sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;
};

struct stpoll_rec {
	struct lwkt_serialize	*serializer;
	struct ifnet		*ifp;
	ifpoll_stfn_t		status_func;
};

struct stpoll_ctx {
	struct netmsg_base	poll_netmsg;

	uint32_t		poll_handlers; /* next free entry in pr[]. */
	struct stpoll_rec	pr[IFPOLL_LIST_LEN];

	struct sysctl_ctx_list	poll_sysctl_ctx;
	struct sysctl_oid	*poll_sysctl_tree;
} __cachealign;

struct iopoll_sysctl_netmsg {
	struct netmsg_base	base;
	struct iopoll_ctx	*ctx;
};

static void	ifpoll_init_pcpu(int);
static void	ifpoll_register_handler(netmsg_t);
static void	ifpoll_deregister_handler(netmsg_t);

/*
 * Status polling
 */
static void	stpoll_init(void);
static void	stpoll_handler(netmsg_t);
static void	stpoll_clock(struct stpoll_ctx *);
static int	stpoll_register(struct ifnet *, const struct ifpoll_status *);
static int	stpoll_deregister(struct ifnet *);

/*
 * RX/TX polling
 */
static struct iopoll_ctx *iopoll_ctx_create(int, int);
static void	iopoll_init(int);
static void	rxpoll_handler(netmsg_t);
static void	txpoll_handler(netmsg_t);
static void	rxpollmore_handler(netmsg_t);
static void	txpollmore_handler(netmsg_t);
static void	iopoll_clock(struct iopoll_ctx *);
static int	iopoll_register(struct ifnet *, struct iopoll_ctx *,
		    const struct ifpoll_io *);
static int	iopoll_deregister(struct ifnet *, struct iopoll_ctx *);

static void	iopoll_add_sysctl(struct sysctl_ctx_list *,
		    struct sysctl_oid_list *, struct iopoll_ctx *, int);
static void	sysctl_burstmax_handler(netmsg_t);
static int	sysctl_burstmax(SYSCTL_HANDLER_ARGS);
static void	sysctl_eachburst_handler(netmsg_t);
static int	sysctl_eachburst(SYSCTL_HANDLER_ARGS);

/*
 * Common functions
 */
static void	poll_comm_init(int);
static void	poll_comm_start(int);
static void	poll_comm_adjust_pollhz(struct poll_comm *);
static void	poll_comm_systimer0(systimer_t, int, struct intrframe *);
static void	poll_comm_systimer(systimer_t, int, struct intrframe *);
static void	sysctl_pollhz_handler(netmsg_t);
static void	sysctl_stfrac_handler(netmsg_t);
static void	sysctl_txfrac_handler(netmsg_t);
static int	sysctl_pollhz(SYSCTL_HANDLER_ARGS);
static int	sysctl_stfrac(SYSCTL_HANDLER_ARGS);
static int	sysctl_txfrac(SYSCTL_HANDLER_ARGS);
static int	sysctl_compat_npoll_stfrac(SYSCTL_HANDLER_ARGS);
static int	sysctl_compat_npoll_cpuid(SYSCTL_HANDLER_ARGS);

static struct stpoll_ctx	stpoll_context;
static struct poll_comm		**poll_common;
static struct iopoll_ctx	**rxpoll_context;
static struct iopoll_ctx	**txpoll_context;

SYSCTL_NODE(_net, OID_AUTO, ifpoll, CTLFLAG_RW, 0,
	    "Network device polling parameters");

static int	iopoll_burst_max = IOPOLL_BURST_MAX;
static int	iopoll_each_burst = IOPOLL_EACH_BURST;
static int	iopoll_user_frac = IOPOLL_USER_FRAC;

static int	ifpoll_pollhz = IFPOLL_FREQ_DEFAULT;
static int	ifpoll_stfrac = IFPOLL_STFRAC_DEFAULT;
static int	ifpoll_txfrac = IFPOLL_TXFRAC_DEFAULT;

TUNABLE_INT("net.ifpoll.burst_max", &iopoll_burst_max);
TUNABLE_INT("net.ifpoll.each_burst", &iopoll_each_burst);
TUNABLE_INT("net.ifpoll.user_frac", &iopoll_user_frac);
TUNABLE_INT("net.ifpoll.pollhz", &ifpoll_pollhz);
TUNABLE_INT("net.ifpoll.status_frac", &ifpoll_stfrac);
TUNABLE_INT("net.ifpoll.tx_frac", &ifpoll_txfrac);

#if !defined(KTR_IF_POLL)
#define  KTR_IF_POLL		KTR_ALL
#endif
KTR_INFO_MASTER(if_poll);
KTR_INFO(KTR_IF_POLL, if_poll, rx_start, 0, "rx start");
KTR_INFO(KTR_IF_POLL, if_poll, rx_end, 1, "rx end");
KTR_INFO(KTR_IF_POLL, if_poll, tx_start, 2, "tx start");
KTR_INFO(KTR_IF_POLL, if_poll, tx_end, 3, "tx end");
KTR_INFO(KTR_IF_POLL, if_poll, rx_mstart, 4, "rx more start");
KTR_INFO(KTR_IF_POLL, if_poll, rx_mend, 5, "rx more end");
KTR_INFO(KTR_IF_POLL, if_poll, tx_mstart, 6, "tx more start");
KTR_INFO(KTR_IF_POLL, if_poll, tx_mend, 7, "tx more end");
KTR_INFO(KTR_IF_POLL, if_poll, ioclock_start, 8, "ioclock start");
KTR_INFO(KTR_IF_POLL, if_poll, ioclock_end, 9, "ioclock end");
#define logpoll(name)	KTR_LOG(if_poll_ ## name)

#define IFPOLL_FREQ_ADJ(comm)	(((comm)->poll_cpuid * 3) % 50)

static __inline int
poll_comm_pollhz_div(const struct poll_comm *comm, int pollhz)
{
	return pollhz + IFPOLL_FREQ_ADJ(comm);
}

static __inline int
poll_comm_pollhz_conv(const struct poll_comm *comm, int pollhz)
{
	return pollhz - IFPOLL_FREQ_ADJ(comm);
}

static __inline void
ifpoll_sendmsg_oncpu(netmsg_t msg)
{
	if (msg->lmsg.ms_flags & MSGF_DONE)
		netisr_sendmsg_oncpu(&msg->base);
}

static __inline void
sched_stpoll(struct stpoll_ctx *st_ctx)
{
	ifpoll_sendmsg_oncpu((netmsg_t)&st_ctx->poll_netmsg);
}

static __inline void
sched_iopoll(struct iopoll_ctx *io_ctx)
{
	ifpoll_sendmsg_oncpu((netmsg_t)&io_ctx->poll_netmsg);
}

static __inline void
sched_iopollmore(struct iopoll_ctx *io_ctx, boolean_t direct)
{

	if (!direct) {
		ifpoll_sendmsg_oncpu((netmsg_t)&io_ctx->poll_more_netmsg);
	} else {
		struct netmsg_base *nmsg = &io_ctx->poll_more_netmsg;

		nmsg->lmsg.ms_flags &= ~(MSGF_REPLY | MSGF_DONE);
		nmsg->lmsg.ms_flags |= MSGF_SYNC;
		nmsg->nm_dispatch((netmsg_t)nmsg);
		KKASSERT(nmsg->lmsg.ms_flags & MSGF_DONE);
	}
}

/*
 * Initialize per-cpu polling(4) context.
 */
static void
ifpoll_init_pcpu(int cpuid)
{

	poll_comm_init(cpuid);

	if (cpuid == 0)
		stpoll_init();
	iopoll_init(cpuid);

	poll_comm_start(cpuid);
}

static void
ifpoll_init_handler(netmsg_t msg)
{
	int cpu = mycpuid;

	ifpoll_init_pcpu(cpu);
	netisr_forwardmsg(&msg->base, cpu + 1);
}

static void
ifpoll_sysinit(void *dummy __unused)
{
	struct netmsg_base msg;

	poll_common = kmalloc(ncpus * sizeof(struct poll_comm *),
	    M_DEVBUF, M_ZERO | M_WAITOK);
	rxpoll_context = kmalloc(ncpus * sizeof(struct iopoll_ctx *),
	    M_DEVBUF, M_ZERO | M_WAITOK);
	txpoll_context = kmalloc(ncpus * sizeof(struct iopoll_ctx *),
	    M_DEVBUF, M_ZERO | M_WAITOK);

	netmsg_init(&msg, NULL, &curthread->td_msgport, 0, ifpoll_init_handler);
	netisr_domsg_global(&msg);
}
SYSINIT(ifpoll, SI_SUB_PRE_DRIVERS, SI_ORDER_ANY, ifpoll_sysinit, NULL);

int
ifpoll_register(struct ifnet *ifp)
{
	struct ifpoll_info *info;
	struct netmsg_base nmsg;
	int error;

	if (ifp->if_npoll == NULL) {
		/* Device does not support polling */
		return EOPNOTSUPP;
	}

	info = kmalloc(sizeof(*info), M_TEMP, M_WAITOK | M_ZERO);

	/*
	 * Attempt to register.  Interlock with IFF_NPOLLING.
	 */

	ifnet_serialize_all(ifp);

	if (ifp->if_flags & IFF_NPOLLING) {
		/* Already polling */
		ifnet_deserialize_all(ifp);
		kfree(info, M_TEMP);
		return EBUSY;
	}

	info->ifpi_ifp = ifp;

	ifp->if_flags |= IFF_NPOLLING;
	ifp->if_npoll(ifp, info);

	ifnet_deserialize_all(ifp);

	netmsg_init(&nmsg, NULL, &curthread->td_msgport,
		    0, ifpoll_register_handler);
	nmsg.lmsg.u.ms_resultp = info;

	error = netisr_domsg_global(&nmsg);
	if (error) {
		if (!ifpoll_deregister(ifp)) {
			if_printf(ifp, "ifpoll_register: "
				  "ifpoll_deregister failed!\n");
		}
	}

	kfree(info, M_TEMP);
	return error;
}

int
ifpoll_deregister(struct ifnet *ifp)
{
	struct netmsg_base nmsg;
	int error;

	if (ifp->if_npoll == NULL)
		return EOPNOTSUPP;

	ifnet_serialize_all(ifp);

	if ((ifp->if_flags & IFF_NPOLLING) == 0) {
		ifnet_deserialize_all(ifp);
		return EINVAL;
	}
	ifp->if_flags &= ~IFF_NPOLLING;

	ifnet_deserialize_all(ifp);

	netmsg_init(&nmsg, NULL, &curthread->td_msgport,
		    0, ifpoll_deregister_handler);
	nmsg.lmsg.u.ms_resultp = ifp;

	error = netisr_domsg_global(&nmsg);
	if (!error) {
		ifnet_serialize_all(ifp);
		ifp->if_npoll(ifp, NULL);
		ifnet_deserialize_all(ifp);
	}
	return error;
}

static void
ifpoll_register_handler(netmsg_t nmsg)
{
	const struct ifpoll_info *info = nmsg->lmsg.u.ms_resultp;
	int cpuid = mycpuid;
	int error;

	KKASSERT(cpuid < netisr_ncpus);
	KKASSERT(&curthread->td_msgport == netisr_cpuport(cpuid));

	if (cpuid == 0) {
		error = stpoll_register(info->ifpi_ifp, &info->ifpi_status);
		if (error)
			goto failed;
	}

	error = iopoll_register(info->ifpi_ifp, rxpoll_context[cpuid],
				&info->ifpi_rx[cpuid]);
	if (error)
		goto failed;

	error = iopoll_register(info->ifpi_ifp, txpoll_context[cpuid],
				&info->ifpi_tx[cpuid]);
	if (error)
		goto failed;

	/* Adjust polling frequency, after all registration is done */
	poll_comm_adjust_pollhz(poll_common[cpuid]);

	netisr_forwardmsg(&nmsg->base, cpuid + 1);
	return;
failed:
	netisr_replymsg(&nmsg->base, error);
}

static void
ifpoll_deregister_handler(netmsg_t nmsg)
{
	struct ifnet *ifp = nmsg->lmsg.u.ms_resultp;
	int cpuid = mycpuid;

	KKASSERT(cpuid < netisr_ncpus);
	KKASSERT(&curthread->td_msgport == netisr_cpuport(cpuid));

	/* Ignore errors */
	if (cpuid == 0)
		stpoll_deregister(ifp);
	iopoll_deregister(ifp, rxpoll_context[cpuid]);
	iopoll_deregister(ifp, txpoll_context[cpuid]);

	/* Adjust polling frequency, after all deregistration is done */
	poll_comm_adjust_pollhz(poll_common[cpuid]);

	netisr_forwardmsg(&nmsg->base, cpuid + 1);
}

static void
stpoll_init(void)
{
	struct stpoll_ctx *st_ctx = &stpoll_context;
	const struct poll_comm *comm = poll_common[0];

	sysctl_ctx_init(&st_ctx->poll_sysctl_ctx);
	st_ctx->poll_sysctl_tree = SYSCTL_ADD_NODE(&st_ctx->poll_sysctl_ctx,
				   SYSCTL_CHILDREN(comm->sysctl_tree),
				   OID_AUTO, "status", CTLFLAG_RD, 0, "");

	SYSCTL_ADD_UINT(&st_ctx->poll_sysctl_ctx,
			SYSCTL_CHILDREN(st_ctx->poll_sysctl_tree),
			OID_AUTO, "handlers", CTLFLAG_RD,
			&st_ctx->poll_handlers, 0,
			"Number of registered status poll handlers");

	netmsg_init(&st_ctx->poll_netmsg, NULL, &netisr_adone_rport,
		    0, stpoll_handler);
}

/*
 * stpoll_handler is scheduled by sched_stpoll when appropriate, typically
 * once per polling systimer tick.
 */
static void
stpoll_handler(netmsg_t msg)
{
	struct stpoll_ctx *st_ctx = &stpoll_context;
	struct thread *td = curthread;
	int i;

	ASSERT_NETISR0;

	crit_enter_quick(td);

	/* Reply ASAP */
	netisr_replymsg(&msg->base, 0);

	if (st_ctx->poll_handlers == 0) {
		crit_exit_quick(td);
		return;
	}

	for (i = 0; i < st_ctx->poll_handlers; ++i) {
		const struct stpoll_rec *rec = &st_ctx->pr[i];
		struct ifnet *ifp = rec->ifp;

		if (!lwkt_serialize_try(rec->serializer))
			continue;

		if ((ifp->if_flags & (IFF_RUNNING | IFF_NPOLLING)) ==
		    (IFF_RUNNING | IFF_NPOLLING))
			rec->status_func(ifp);

		lwkt_serialize_exit(rec->serializer);
	}

	crit_exit_quick(td);
}

/*
 * Hook from status poll systimer.  Tries to schedule an status poll.
 * NOTE: Caller should hold critical section.
 */
static void
stpoll_clock(struct stpoll_ctx *st_ctx)
{
	KKASSERT(mycpuid == 0);

	if (st_ctx->poll_handlers == 0)
		return;
	sched_stpoll(st_ctx);
}

static int
stpoll_register(struct ifnet *ifp, const struct ifpoll_status *st_rec)
{
	struct stpoll_ctx *st_ctx = &stpoll_context;
	int error;

	ASSERT_NETISR0;

	if (st_rec->status_func == NULL)
		return 0;

	/*
	 * Check if there is room.
	 */
	if (st_ctx->poll_handlers >= IFPOLL_LIST_LEN) {
		/*
		 * List full, cannot register more entries.
		 * This should never happen; if it does, it is probably a
		 * broken driver trying to register multiple times. Checking
		 * this at runtime is expensive, and won't solve the problem
		 * anyways, so just report a few times and then give up.
		 */
		static int verbose = 10; /* XXX */

		if (verbose > 0) {
			kprintf("status poll handlers list full, "
				"maybe a broken driver ?\n");
			verbose--;
		}
		error = ENOENT;
	} else {
		struct stpoll_rec *rec = &st_ctx->pr[st_ctx->poll_handlers];

		rec->ifp = ifp;
		rec->serializer = st_rec->serializer;
		rec->status_func = st_rec->status_func;

		st_ctx->poll_handlers++;
		error = 0;
	}
	return error;
}

static int
stpoll_deregister(struct ifnet *ifp)
{
	struct stpoll_ctx *st_ctx = &stpoll_context;
	int i, error;

	ASSERT_NETISR0;

	for (i = 0; i < st_ctx->poll_handlers; ++i) {
		if (st_ctx->pr[i].ifp == ifp) /* Found it */
			break;
	}
	if (i == st_ctx->poll_handlers) {
		error = ENOENT;
	} else {
		st_ctx->poll_handlers--;
		if (i < st_ctx->poll_handlers) {
			/* Last entry replaces this one. */
			st_ctx->pr[i] = st_ctx->pr[st_ctx->poll_handlers];
		}
		error = 0;
	}
	return error;
}

static __inline void
iopoll_reset_state(struct iopoll_ctx *io_ctx)
{
	crit_enter();
	io_ctx->poll_burst = io_ctx->poll_each_burst;
	io_ctx->pending_polls = 0;
	io_ctx->residual_burst = 0;
	io_ctx->phase = 0;
	io_ctx->kern_frac = 0;
	bzero(&io_ctx->poll_start_t, sizeof(io_ctx->poll_start_t));
	bzero(&io_ctx->prev_t, sizeof(io_ctx->prev_t));
	crit_exit();
}

static void
iopoll_init(int cpuid)
{
	KKASSERT(cpuid < netisr_ncpus);

	rxpoll_context[cpuid] = iopoll_ctx_create(cpuid, IFPOLL_RX);
	txpoll_context[cpuid] = iopoll_ctx_create(cpuid, IFPOLL_TX);
}

static struct iopoll_ctx *
iopoll_ctx_create(int cpuid, int poll_type)
{
	struct poll_comm *comm;
	struct iopoll_ctx *io_ctx;
	const char *poll_type_str;
	netisr_fn_t handler, more_handler;

	KKASSERT(poll_type == IFPOLL_RX || poll_type == IFPOLL_TX);

	/*
	 * Make sure that tunables are in sane state
	 */
	if (iopoll_burst_max < MIN_IOPOLL_BURST_MAX)
		iopoll_burst_max = MIN_IOPOLL_BURST_MAX;
	else if (iopoll_burst_max > MAX_IOPOLL_BURST_MAX)
		iopoll_burst_max = MAX_IOPOLL_BURST_MAX;

	if (iopoll_each_burst > iopoll_burst_max)
		iopoll_each_burst = iopoll_burst_max;

	comm = poll_common[cpuid];

	/*
	 * Create the per-cpu polling context
	 */
	io_ctx = kmalloc(sizeof(*io_ctx), M_DEVBUF, M_WAITOK | M_ZERO);

	io_ctx->poll_each_burst = iopoll_each_burst;
	io_ctx->poll_burst_max = iopoll_burst_max;
	io_ctx->user_frac = iopoll_user_frac;
	if (poll_type == IFPOLL_RX)
		io_ctx->pollhz = comm->pollhz;
	else
		io_ctx->pollhz = comm->pollhz / (comm->poll_txfrac + 1);
	io_ctx->poll_cpuid = cpuid;
	iopoll_reset_state(io_ctx);

	if (poll_type == IFPOLL_RX) {
		handler = rxpoll_handler;
		more_handler = rxpollmore_handler;
	} else {
		handler = txpoll_handler;
		more_handler = txpollmore_handler;
	}

	netmsg_init(&io_ctx->poll_netmsg, NULL, &netisr_adone_rport,
	    0, handler);
	io_ctx->poll_netmsg.lmsg.u.ms_resultp = io_ctx;

	netmsg_init(&io_ctx->poll_more_netmsg, NULL, &netisr_adone_rport,
	    0, more_handler);
	io_ctx->poll_more_netmsg.lmsg.u.ms_resultp = io_ctx;

	/*
	 * Initialize per-cpu sysctl nodes
	 */
	if (poll_type == IFPOLL_RX)
		poll_type_str = "rx";
	else
		poll_type_str = "tx";

	sysctl_ctx_init(&io_ctx->poll_sysctl_ctx);
	io_ctx->poll_sysctl_tree = SYSCTL_ADD_NODE(&io_ctx->poll_sysctl_ctx,
				   SYSCTL_CHILDREN(comm->sysctl_tree),
				   OID_AUTO, poll_type_str, CTLFLAG_RD, 0, "");
	iopoll_add_sysctl(&io_ctx->poll_sysctl_ctx,
	    SYSCTL_CHILDREN(io_ctx->poll_sysctl_tree), io_ctx, poll_type);

	return io_ctx;
}

/*
 * Hook from iopoll systimer.  Tries to schedule an iopoll, but keeps
 * track of lost ticks due to the previous handler taking too long.
 * Normally, this should not happen, because polling handler should
 * run for a short time.  However, in some cases (e.g. when there are
 * changes in link status etc.) the drivers take a very long time
 * (even in the order of milliseconds) to reset and reconfigure the
 * device, causing apparent lost polls.
 *
 * The first part of the code is just for debugging purposes, and tries
 * to count how often hardclock ticks are shorter than they should,
 * meaning either stray interrupts or delayed events.
 *
 * WARNING! called from fastint or IPI, the MP lock might not be held.
 * NOTE: Caller should hold critical section.
 */
static void
iopoll_clock(struct iopoll_ctx *io_ctx)
{
	union microtime_pcpu t;
	int delta;

	KKASSERT(mycpuid == io_ctx->poll_cpuid);

	if (io_ctx->poll_handlers == 0)
		return;

	logpoll(ioclock_start);

	microtime_pcpu_get(&t);
	delta = microtime_pcpu_diff(&io_ctx->prev_t, &t);
	if (delta * io_ctx->pollhz < 500000)
		io_ctx->short_ticks++;
	else
		io_ctx->prev_t = t;

	if (io_ctx->pending_polls > 100) {
		/*
		 * Too much, assume it has stalled (not always true
		 * see comment above).
		 */
		io_ctx->stalled++;
		io_ctx->pending_polls = 0;
		io_ctx->phase = 0;
	}

	if (io_ctx->phase <= 2) {
		if (io_ctx->phase != 0)
			io_ctx->suspect++;
		io_ctx->phase = 1;
		sched_iopoll(io_ctx);
		io_ctx->phase = 2;
	}
	if (io_ctx->pending_polls++ > 0)
		io_ctx->lost_polls++;

	logpoll(ioclock_end);
}

/*
 * rxpoll_handler and txpoll_handler are scheduled by sched_iopoll when
 * appropriate, typically once per polling systimer tick.
 *
 * Note that the message is replied immediately in order to allow a new
 * ISR to be scheduled in the handler.
 */
static void
rxpoll_handler(netmsg_t msg)
{
	struct iopoll_ctx *io_ctx;
	struct thread *td = curthread;
	boolean_t direct = TRUE, crit;
	int i, cycles;

	logpoll(rx_start);

	io_ctx = msg->lmsg.u.ms_resultp;
	KKASSERT(&td->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	crit = TRUE;
	crit_enter_quick(td);

	/* Reply ASAP */
	netisr_replymsg(&msg->base, 0);

	if (io_ctx->poll_handlers == 0) {
		crit_exit_quick(td);
		logpoll(rx_end);
		return;
	}

	io_ctx->phase = 3;
	if (io_ctx->residual_burst == 0) {
		/* First call in this tick */
		microtime_pcpu_get(&io_ctx->poll_start_t);
		io_ctx->residual_burst = io_ctx->poll_burst;
	}
	cycles = (io_ctx->residual_burst < io_ctx->poll_each_burst) ?
		 io_ctx->residual_burst : io_ctx->poll_each_burst;
	io_ctx->residual_burst -= cycles;

	for (i = 0; i < io_ctx->poll_handlers; i++) {
		const struct iopoll_rec *rec = &io_ctx->pr[i];
		struct ifnet *ifp = rec->ifp;

		if (rec->serializer != NULL) {
			if (!crit) {
				crit = TRUE;
				crit_enter_quick(td);
			}
			if (__predict_false(
			    !lwkt_serialize_try(rec->serializer))) {
				/* RX serializer generally will not fail. */
				continue;
			}
		} else if (crit) {
			/*
			 * Exit critical section, if the RX polling
			 * handler does not require serialization,
			 * i.e. RX polling is doing direct input.
			 */
			crit_exit_quick(td);
			crit = FALSE;
		}

		if ((ifp->if_flags & IFF_IDIRECT) == 0) {
			direct = FALSE;
			KASSERT(rec->serializer != NULL,
			    ("rx polling handler is not serialized"));
		}
#ifdef INVARIANTS
		else {
			KASSERT(rec->serializer == NULL,
			    ("serialized direct input"));
		}
#endif

		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING | IFF_NPOLLING)) ==
		    (IFF_UP | IFF_RUNNING | IFF_NPOLLING))
			rec->poll_func(ifp, rec->arg, cycles);

		if (rec->serializer != NULL)
			lwkt_serialize_exit(rec->serializer);
	}

	if (crit) {
		/*
		 * Do a quick exit/enter to catch any higher-priority
		 * interrupt sources.
		 */
		crit_exit_quick(td);
	}
	crit_enter_quick(td);

	io_ctx->phase = 4;
	sched_iopollmore(io_ctx, direct);

	crit_exit_quick(td);

	logpoll(rx_end);
}

static void
txpoll_handler(netmsg_t msg)
{
	struct iopoll_ctx *io_ctx;
	struct thread *td = curthread;
	int i;

	logpoll(tx_start);

	io_ctx = msg->lmsg.u.ms_resultp;
	KKASSERT(&td->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	crit_enter_quick(td);

	/* Reply ASAP */
	netisr_replymsg(&msg->base, 0);

	if (io_ctx->poll_handlers == 0) {
		crit_exit_quick(td);
		logpoll(tx_end);
		return;
	}

	io_ctx->phase = 3;

	for (i = 0; i < io_ctx->poll_handlers; i++) {
		const struct iopoll_rec *rec = &io_ctx->pr[i];
		struct ifnet *ifp = rec->ifp;

		if (!lwkt_serialize_try(rec->serializer))
			continue;

		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING | IFF_NPOLLING)) ==
		    (IFF_UP | IFF_RUNNING | IFF_NPOLLING))
			rec->poll_func(ifp, rec->arg, -1);

		lwkt_serialize_exit(rec->serializer);
	}

	/*
	 * Do a quick exit/enter to catch any higher-priority
	 * interrupt sources.
	 */
	crit_exit_quick(td);
	crit_enter_quick(td);

	io_ctx->phase = 4;
	sched_iopollmore(io_ctx, TRUE);

	crit_exit_quick(td);

	logpoll(tx_end);
}

/*
 * rxpollmore_handler and txpollmore_handler are called after other netisr's,
 * possibly scheduling another rxpoll_handler or txpoll_handler call, or
 * adapting the burst size for the next cycle.
 *
 * It is very bad to fetch large bursts of packets from a single card at once,
 * because the burst could take a long time to be completely processed leading
 * to unfairness.  To reduce the problem, and also to account better for time
 * spent in network-related processing, we split the burst in smaller chunks
 * of fixed size, giving control to the other netisr's between chunks.  This
 * helps in improving the fairness, reducing livelock and accounting for the
 * work performed in low level handling.
 */
static void
rxpollmore_handler(netmsg_t msg)
{
	struct thread *td = curthread;
	struct iopoll_ctx *io_ctx;
	union microtime_pcpu t;
	int kern_load;
	uint32_t pending_polls;

	logpoll(rx_mstart);

	io_ctx = msg->lmsg.u.ms_resultp;
	KKASSERT(&td->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	crit_enter_quick(td);

	/* Replay ASAP */
	netisr_replymsg(&msg->base, 0);

	if (io_ctx->poll_handlers == 0) {
		crit_exit_quick(td);
		logpoll(rx_mend);
		return;
	}

	io_ctx->phase = 5;
	if (io_ctx->residual_burst > 0) {
		sched_iopoll(io_ctx);
		crit_exit_quick(td);
		/* Will run immediately on return, followed by netisrs */
		logpoll(rx_mend);
		return;
	}

	/* Here we can account time spent in iopoll's in this tick */
	microtime_pcpu_get(&t);
	kern_load = microtime_pcpu_diff(&io_ctx->poll_start_t, &t);
	kern_load = (kern_load * io_ctx->pollhz) / 10000; /* 0..100 */
	io_ctx->kern_frac = kern_load;

	if (kern_load > (100 - io_ctx->user_frac)) {
		/* Try decrease ticks */
		if (io_ctx->poll_burst > 1)
			io_ctx->poll_burst--;
	} else {
		if (io_ctx->poll_burst < io_ctx->poll_burst_max)
			io_ctx->poll_burst++;
	}

	io_ctx->pending_polls--;
	pending_polls = io_ctx->pending_polls;

	if (pending_polls == 0) {
		/* We are done */
		io_ctx->phase = 0;
	} else {
		/*
		 * Last cycle was long and caused us to miss one or more
		 * hardclock ticks.  Restart processing again, but slightly
		 * reduce the burst size to prevent that this happens again.
		 */
		io_ctx->poll_burst -= (io_ctx->poll_burst / 8);
		if (io_ctx->poll_burst < 1)
			io_ctx->poll_burst = 1;
		sched_iopoll(io_ctx);
		io_ctx->phase = 6;
	}

	crit_exit_quick(td);

	logpoll(rx_mend);
}

static void
txpollmore_handler(netmsg_t msg)
{
	struct thread *td = curthread;
	struct iopoll_ctx *io_ctx;
	uint32_t pending_polls;

	logpoll(tx_mstart);

	io_ctx = msg->lmsg.u.ms_resultp;
	KKASSERT(&td->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	crit_enter_quick(td);

	/* Replay ASAP */
	netisr_replymsg(&msg->base, 0);

	if (io_ctx->poll_handlers == 0) {
		crit_exit_quick(td);
		logpoll(tx_mend);
		return;
	}

	io_ctx->phase = 5;

	io_ctx->pending_polls--;
	pending_polls = io_ctx->pending_polls;

	if (pending_polls == 0) {
		/* We are done */
		io_ctx->phase = 0;
	} else {
		/*
		 * Last cycle was long and caused us to miss one or more
		 * hardclock ticks.  Restart processing again.
		 */
		sched_iopoll(io_ctx);
		io_ctx->phase = 6;
	}

	crit_exit_quick(td);

	logpoll(tx_mend);
}

static void
iopoll_add_sysctl(struct sysctl_ctx_list *ctx, struct sysctl_oid_list *parent,
    struct iopoll_ctx *io_ctx, int poll_type)
{
	if (poll_type == IFPOLL_RX) {
		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO, "burst_max",
		    CTLTYPE_UINT | CTLFLAG_RW, io_ctx, 0, sysctl_burstmax,
		    "IU", "Max Polling burst size");

		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO, "each_burst",
		    CTLTYPE_UINT | CTLFLAG_RW, io_ctx, 0, sysctl_eachburst,
		    "IU", "Max size of each burst");

		SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "burst", CTLFLAG_RD,
		    &io_ctx->poll_burst, 0, "Current polling burst size");

		SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "user_frac", CTLFLAG_RW,
		    &io_ctx->user_frac, 0, "Desired user fraction of cpu time");

		SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "kern_frac", CTLFLAG_RD,
		    &io_ctx->kern_frac, 0, "Kernel fraction of cpu time");

		SYSCTL_ADD_INT(ctx, parent, OID_AUTO, "residual_burst", CTLFLAG_RD,
		    &io_ctx->residual_burst, 0,
		    "# of residual cycles in burst");
	}

	SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "phase", CTLFLAG_RD,
	    &io_ctx->phase, 0, "Polling phase");

	SYSCTL_ADD_ULONG(ctx, parent, OID_AUTO, "suspect", CTLFLAG_RW,
	    &io_ctx->suspect, "Suspected events");

	SYSCTL_ADD_ULONG(ctx, parent, OID_AUTO, "stalled", CTLFLAG_RW,
	    &io_ctx->stalled, "Potential stalls");

	SYSCTL_ADD_ULONG(ctx, parent, OID_AUTO, "short_ticks", CTLFLAG_RW,
	    &io_ctx->short_ticks,
	    "Hardclock ticks shorter than they should be");

	SYSCTL_ADD_ULONG(ctx, parent, OID_AUTO, "lost_polls", CTLFLAG_RW,
	    &io_ctx->lost_polls,
	    "How many times we would have lost a poll tick");

	SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "pending_polls", CTLFLAG_RD,
	    &io_ctx->pending_polls, 0, "Do we need to poll again");

	SYSCTL_ADD_UINT(ctx, parent, OID_AUTO, "handlers", CTLFLAG_RD,
	    &io_ctx->poll_handlers, 0, "Number of registered poll handlers");
}

static void
sysctl_burstmax_handler(netmsg_t nmsg)
{
	struct iopoll_sysctl_netmsg *msg = (struct iopoll_sysctl_netmsg *)nmsg;
	struct iopoll_ctx *io_ctx;

	io_ctx = msg->ctx;
	KKASSERT(&curthread->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	io_ctx->poll_burst_max = nmsg->lmsg.u.ms_result;
	if (io_ctx->poll_each_burst > io_ctx->poll_burst_max)
		io_ctx->poll_each_burst = io_ctx->poll_burst_max;
	if (io_ctx->poll_burst > io_ctx->poll_burst_max)
		io_ctx->poll_burst = io_ctx->poll_burst_max;
	if (io_ctx->residual_burst > io_ctx->poll_burst_max)
		io_ctx->residual_burst = io_ctx->poll_burst_max;

	netisr_replymsg(&nmsg->base, 0);
}

static int
sysctl_burstmax(SYSCTL_HANDLER_ARGS)
{
	struct iopoll_ctx *io_ctx = arg1;
	struct iopoll_sysctl_netmsg msg;
	uint32_t burst_max;
	int error;

	burst_max = io_ctx->poll_burst_max;
	error = sysctl_handle_int(oidp, &burst_max, 0, req);
	if (error || req->newptr == NULL)
		return error;
	if (burst_max < MIN_IOPOLL_BURST_MAX)
		burst_max = MIN_IOPOLL_BURST_MAX;
	else if (burst_max > MAX_IOPOLL_BURST_MAX)
		burst_max = MAX_IOPOLL_BURST_MAX;

	netmsg_init(&msg.base, NULL, &curthread->td_msgport,
		    0, sysctl_burstmax_handler);
	msg.base.lmsg.u.ms_result = burst_max;
	msg.ctx = io_ctx;

	return netisr_domsg(&msg.base, io_ctx->poll_cpuid);
}

static void
sysctl_eachburst_handler(netmsg_t nmsg)
{
	struct iopoll_sysctl_netmsg *msg = (struct iopoll_sysctl_netmsg *)nmsg;
	struct iopoll_ctx *io_ctx;
	uint32_t each_burst;

	io_ctx = msg->ctx;
	KKASSERT(&curthread->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	each_burst = nmsg->lmsg.u.ms_result;
	if (each_burst > io_ctx->poll_burst_max)
		each_burst = io_ctx->poll_burst_max;
	else if (each_burst < 1)
		each_burst = 1;
	io_ctx->poll_each_burst = each_burst;

	netisr_replymsg(&nmsg->base, 0);
}

static int
sysctl_eachburst(SYSCTL_HANDLER_ARGS)
{
	struct iopoll_ctx *io_ctx = arg1;
	struct iopoll_sysctl_netmsg msg;
	uint32_t each_burst;
	int error;

	each_burst = io_ctx->poll_each_burst;
	error = sysctl_handle_int(oidp, &each_burst, 0, req);
	if (error || req->newptr == NULL)
		return error;

	netmsg_init(&msg.base, NULL, &curthread->td_msgport,
		    0, sysctl_eachburst_handler);
	msg.base.lmsg.u.ms_result = each_burst;
	msg.ctx = io_ctx;

	return netisr_domsg(&msg.base, io_ctx->poll_cpuid);
}

static int
iopoll_register(struct ifnet *ifp, struct iopoll_ctx *io_ctx,
		const struct ifpoll_io *io_rec)
{
	int error;

	KKASSERT(&curthread->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	if (io_rec->poll_func == NULL)
		return 0;

	/*
	 * Check if there is room.
	 */
	if (io_ctx->poll_handlers >= IFPOLL_LIST_LEN) {
		/*
		 * List full, cannot register more entries.
		 * This should never happen; if it does, it is probably a
		 * broken driver trying to register multiple times. Checking
		 * this at runtime is expensive, and won't solve the problem
		 * anyways, so just report a few times and then give up.
		 */
		static int verbose = 10; /* XXX */
		if (verbose > 0) {
			kprintf("io poll handlers list full, "
				"maybe a broken driver ?\n");
			verbose--;
		}
		error = ENOENT;
	} else {
		struct iopoll_rec *rec = &io_ctx->pr[io_ctx->poll_handlers];

		rec->ifp = ifp;
		rec->serializer = io_rec->serializer;
		rec->arg = io_rec->arg;
		rec->poll_func = io_rec->poll_func;

		io_ctx->poll_handlers++;
		error = 0;
	}
	return error;
}

static int
iopoll_deregister(struct ifnet *ifp, struct iopoll_ctx *io_ctx)
{
	int i, error;

	KKASSERT(&curthread->td_msgport == netisr_cpuport(io_ctx->poll_cpuid));

	for (i = 0; i < io_ctx->poll_handlers; ++i) {
		if (io_ctx->pr[i].ifp == ifp) /* Found it */
			break;
	}
	if (i == io_ctx->poll_handlers) {
		error = ENOENT;
	} else {
		io_ctx->poll_handlers--;
		if (i < io_ctx->poll_handlers) {
			/* Last entry replaces this one. */
			io_ctx->pr[i] = io_ctx->pr[io_ctx->poll_handlers];
		}

		if (io_ctx->poll_handlers == 0)
			iopoll_reset_state(io_ctx);
		error = 0;
	}
	return error;
}

static void
poll_comm_init(int cpuid)
{
	struct poll_comm *comm;
	char cpuid_str[16];

	comm = kmalloc(sizeof(*comm), M_DEVBUF, M_WAITOK | M_ZERO);

	if (ifpoll_stfrac < 1)
		ifpoll_stfrac = IFPOLL_STFRAC_DEFAULT;
	if (ifpoll_txfrac < 1)
		ifpoll_txfrac = IFPOLL_TXFRAC_DEFAULT;

	comm->poll_cpuid = cpuid;
	comm->pollhz = poll_comm_pollhz_div(comm, ifpoll_pollhz);
	comm->poll_stfrac = ifpoll_stfrac - 1;
	comm->poll_txfrac = ifpoll_txfrac - 1;

	ksnprintf(cpuid_str, sizeof(cpuid_str), "%d", cpuid);

	sysctl_ctx_init(&comm->sysctl_ctx);
	comm->sysctl_tree = SYSCTL_ADD_NODE(&comm->sysctl_ctx,
			    SYSCTL_STATIC_CHILDREN(_net_ifpoll),
			    OID_AUTO, cpuid_str, CTLFLAG_RD, 0, "");

	SYSCTL_ADD_PROC(&comm->sysctl_ctx, SYSCTL_CHILDREN(comm->sysctl_tree),
			OID_AUTO, "pollhz", CTLTYPE_INT | CTLFLAG_RW,
			comm, 0, sysctl_pollhz,
			"I", "Device polling frequency");

	if (cpuid == 0) {
		SYSCTL_ADD_PROC(&comm->sysctl_ctx,
				SYSCTL_CHILDREN(comm->sysctl_tree),
				OID_AUTO, "status_frac",
				CTLTYPE_INT | CTLFLAG_RW,
				comm, 0, sysctl_stfrac,
				"I", "# of cycles before status is polled");
	}
	SYSCTL_ADD_PROC(&comm->sysctl_ctx, SYSCTL_CHILDREN(comm->sysctl_tree),
			OID_AUTO, "tx_frac", CTLTYPE_INT | CTLFLAG_RW,
			comm, 0, sysctl_txfrac,
			"I", "# of cycles before TX is polled");

	poll_common[cpuid] = comm;
}

static void
poll_comm_start(int cpuid)
{
	struct poll_comm *comm = poll_common[cpuid];
	systimer_func_t func;

	/*
	 * Initialize systimer
	 */
	if (cpuid == 0)
		func = poll_comm_systimer0;
	else
		func = poll_comm_systimer;
	systimer_init_periodic_nq(&comm->pollclock, func, comm, 1);
}

static void
_poll_comm_systimer(struct poll_comm *comm)
{
	iopoll_clock(rxpoll_context[comm->poll_cpuid]);
	if (comm->txfrac_count-- == 0) {
		comm->txfrac_count = comm->poll_txfrac;
		iopoll_clock(txpoll_context[comm->poll_cpuid]);
	}
}

static void
poll_comm_systimer0(systimer_t info, int in_ipi __unused,
    struct intrframe *frame __unused)
{
	struct poll_comm *comm = info->data;
	globaldata_t gd = mycpu;

	KKASSERT(comm->poll_cpuid == gd->gd_cpuid && gd->gd_cpuid == 0);

	crit_enter_gd(gd);

	if (comm->stfrac_count-- == 0) {
		comm->stfrac_count = comm->poll_stfrac;
		stpoll_clock(&stpoll_context);
	}
	_poll_comm_systimer(comm);

	crit_exit_gd(gd);
}

static void
poll_comm_systimer(systimer_t info, int in_ipi __unused,
    struct intrframe *frame __unused)
{
	struct poll_comm *comm = info->data;
	globaldata_t gd = mycpu;

	KKASSERT(comm->poll_cpuid == gd->gd_cpuid && gd->gd_cpuid != 0);

	crit_enter_gd(gd);
	_poll_comm_systimer(comm);
	crit_exit_gd(gd);
}

static void
poll_comm_adjust_pollhz(struct poll_comm *comm)
{
	uint32_t handlers;
	int pollhz = 1;

	KKASSERT(&curthread->td_msgport == netisr_cpuport(comm->poll_cpuid));

	/*
	 * If there is no polling handler registered, set systimer
	 * frequency to the lowest value.  Polling systimer frequency
	 * will be adjusted to the requested value, once there are
	 * registered handlers.
	 */
	handlers = rxpoll_context[mycpuid]->poll_handlers +
		   txpoll_context[mycpuid]->poll_handlers;
	if (comm->poll_cpuid == 0)
		handlers += stpoll_context.poll_handlers;
	if (handlers)
		pollhz = comm->pollhz;
	systimer_adjust_periodic(&comm->pollclock, pollhz);
}

static int
sysctl_pollhz(SYSCTL_HANDLER_ARGS)
{
	struct poll_comm *comm = arg1;
	struct netmsg_base nmsg;
	int error, phz;

	phz = poll_comm_pollhz_conv(comm, comm->pollhz);
	error = sysctl_handle_int(oidp, &phz, 0, req);
	if (error || req->newptr == NULL)
		return error;
	if (phz <= 0)
		return EINVAL;
	else if (phz > IFPOLL_FREQ_MAX)
		phz = IFPOLL_FREQ_MAX;

	netmsg_init(&nmsg, NULL, &curthread->td_msgport,
		    0, sysctl_pollhz_handler);
	nmsg.lmsg.u.ms_result = phz;

	return netisr_domsg(&nmsg, comm->poll_cpuid);
}

static void
sysctl_pollhz_handler(netmsg_t nmsg)
{
	struct poll_comm *comm = poll_common[mycpuid];

	KKASSERT(&curthread->td_msgport == netisr_cpuport(comm->poll_cpuid));

	/* Save polling frequency */
	comm->pollhz = poll_comm_pollhz_div(comm, nmsg->lmsg.u.ms_result);

	/*
	 * Adjust cached pollhz
	 */
	rxpoll_context[mycpuid]->pollhz = comm->pollhz;
	txpoll_context[mycpuid]->pollhz =
	    comm->pollhz / (comm->poll_txfrac + 1);

	/*
	 * Adjust polling frequency
	 */
	poll_comm_adjust_pollhz(comm);

	netisr_replymsg(&nmsg->base, 0);
}

static int
sysctl_stfrac(SYSCTL_HANDLER_ARGS)
{
	struct poll_comm *comm = arg1;
	struct netmsg_base nmsg;
	int error, stfrac;

	KKASSERT(comm->poll_cpuid == 0);

	stfrac = comm->poll_stfrac + 1;
	error = sysctl_handle_int(oidp, &stfrac, 0, req);
	if (error || req->newptr == NULL)
		return error;
	if (stfrac < 1)
		return EINVAL;

	netmsg_init(&nmsg, NULL, &curthread->td_msgport,
		    0, sysctl_stfrac_handler);
	nmsg.lmsg.u.ms_result = stfrac - 1;

	return netisr_domsg(&nmsg, comm->poll_cpuid);
}

static void
sysctl_stfrac_handler(netmsg_t nmsg)
{
	struct poll_comm *comm = poll_common[mycpuid];
	int stfrac = nmsg->lmsg.u.ms_result;

	KKASSERT(&curthread->td_msgport == netisr_cpuport(comm->poll_cpuid));

	crit_enter();
	comm->poll_stfrac = stfrac;
	if (comm->stfrac_count > comm->poll_stfrac)
		comm->stfrac_count = comm->poll_stfrac;
	crit_exit();

	netisr_replymsg(&nmsg->base, 0);
}

static int
sysctl_txfrac(SYSCTL_HANDLER_ARGS)
{
	struct poll_comm *comm = arg1;
	struct netmsg_base nmsg;
	int error, txfrac;

	txfrac = comm->poll_txfrac + 1;
	error = sysctl_handle_int(oidp, &txfrac, 0, req);
	if (error || req->newptr == NULL)
		return error;
	if (txfrac < 1)
		return EINVAL;

	netmsg_init(&nmsg, NULL, &curthread->td_msgport,
		    0, sysctl_txfrac_handler);
	nmsg.lmsg.u.ms_result = txfrac - 1;

	return netisr_domsg(&nmsg, comm->poll_cpuid);
}

static void
sysctl_txfrac_handler(netmsg_t nmsg)
{
	struct poll_comm *comm = poll_common[mycpuid];
	int txfrac = nmsg->lmsg.u.ms_result;

	KKASSERT(&curthread->td_msgport == netisr_cpuport(comm->poll_cpuid));

	crit_enter();
	comm->poll_txfrac = txfrac;
	if (comm->txfrac_count > comm->poll_txfrac)
		comm->txfrac_count = comm->poll_txfrac;
	crit_exit();

	netisr_replymsg(&nmsg->base, 0);
}

void
ifpoll_compat_setup(struct ifpoll_compat *cp,
    struct sysctl_ctx_list *sysctl_ctx,
    struct sysctl_oid *sysctl_tree,
    int unit, struct lwkt_serialize *slz)
{
	cp->ifpc_stcount = 0;
	cp->ifpc_stfrac = ((poll_common[0]->poll_stfrac + 1) *
	    howmany(IOPOLL_BURST_MAX, IOPOLL_EACH_BURST)) - 1;

	cp->ifpc_cpuid = unit % netisr_ncpus;
	cp->ifpc_serializer = slz;

	if (sysctl_ctx != NULL && sysctl_tree != NULL) {
		SYSCTL_ADD_PROC(sysctl_ctx, SYSCTL_CHILDREN(sysctl_tree),
		    OID_AUTO, "npoll_stfrac", CTLTYPE_INT | CTLFLAG_RW,
		    cp, 0, sysctl_compat_npoll_stfrac, "I",
		    "polling status frac");
		SYSCTL_ADD_PROC(sysctl_ctx, SYSCTL_CHILDREN(sysctl_tree),
		    OID_AUTO, "npoll_cpuid", CTLTYPE_INT | CTLFLAG_RW,
		    cp, 0, sysctl_compat_npoll_cpuid, "I",
		    "polling cpuid");
	}
}

static int
sysctl_compat_npoll_stfrac(SYSCTL_HANDLER_ARGS)
{
	struct ifpoll_compat *cp = arg1;
	int error = 0, stfrac;

	lwkt_serialize_enter(cp->ifpc_serializer);

	stfrac = cp->ifpc_stfrac + 1;
	error = sysctl_handle_int(oidp, &stfrac, 0, req);
	if (!error && req->newptr != NULL) {
		if (stfrac < 1) {
			error = EINVAL;
		} else {
			cp->ifpc_stfrac = stfrac - 1;
			if (cp->ifpc_stcount > cp->ifpc_stfrac)
				cp->ifpc_stcount = cp->ifpc_stfrac;
		}
	}

	lwkt_serialize_exit(cp->ifpc_serializer);
	return error;
}

static int
sysctl_compat_npoll_cpuid(SYSCTL_HANDLER_ARGS)
{
	struct ifpoll_compat *cp = arg1;
	int error = 0, cpuid;

	lwkt_serialize_enter(cp->ifpc_serializer);

	cpuid = cp->ifpc_cpuid;
	error = sysctl_handle_int(oidp, &cpuid, 0, req);
	if (!error && req->newptr != NULL) {
		if (cpuid < 0 || cpuid >= netisr_ncpus)
			error = EINVAL;
		else
			cp->ifpc_cpuid = cpuid;
	}

	lwkt_serialize_exit(cp->ifpc_serializer);
	return error;
}
