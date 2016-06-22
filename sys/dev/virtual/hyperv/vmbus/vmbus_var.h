/*-
 * Copyright (c) 2016 Microsoft Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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
 *
 * $FreeBSD$
 */

#ifndef _VMBUS_VAR_H_
#define _VMBUS_VAR_H_

#include <sys/param.h>

struct vmbus_pcpu_data {
	struct vmbus_message	*message;	/* shared messages */
	int			event_flags_cnt;/* # of event flags */
	struct vmbus_evtflags	*event_flags;	/* event flags from host */

	struct vmbus_softc	*sc;
	int			cpuid;
	int			vcpuid;

	uint64_t		timer_last;

	/*
	 * Rarely used fields
	 */

	struct hyperv_dma	message_dma;	/* busdma glue */
	struct hyperv_dma	event_flags_dma;/* busdma glue */

	/* Interrupt stuffs */
	void			*intr_hand;
	struct resource		*intr_res;
	int			intr_rid;
	int			intr_vec;
	char			intr_desc[24];
} __cachealign;

struct vmbus_softc {
	u_long			*vmbus_tx_evtflags;
						/* event flags to host */
	void			*vmbus_mnf2;	/* monitored by host */

	u_long			*vmbus_rx_evtflags;
						/* compat evtflgs from host */
	struct vmbus_pcpu_data	vmbus_pcpu[MAXCPU];

	/*
	 * Rarely used fields
	 */

	device_t		vmbus_dev;
	struct vmbus_msghc_ctx	*vmbus_msg_hc;
	uint32_t		vmbus_flags;	/* see VMBUS_FLAG_ */
	uint32_t		vmbus_version;	/* VMBUS_VERSION_ */

	/* Shared memory for vmbus_{rx,tx}_evtflags */
	void			*vmbus_evtflags;
	struct hyperv_dma	vmbus_evtflags_dma;

	void			*vmbus_mnf1;	/* monitored by VM, unused */
	struct hyperv_dma	vmbus_mnf1_dma;
	struct hyperv_dma	vmbus_mnf2_dma;
};

#define VMBUS_FLAG_SYNIC	0x0002	/* SynIC was setup */

#define VMBUS_PCPU(sc, cpu)	&(sc)->vmbus_pcpu[(cpu)]

struct vmbus_message;
struct vmbus_msghc;

struct vmbus_msghc *vmbus_msghc_get(struct vmbus_softc *, size_t);
void	vmbus_msghc_put(struct vmbus_softc *, struct vmbus_msghc *);
void	*vmbus_msghc_dataptr(struct vmbus_msghc *);
int	vmbus_msghc_exec_noresult(struct vmbus_msghc *);
int	vmbus_msghc_exec(struct vmbus_softc *, struct vmbus_msghc *);
const struct vmbus_message *vmbus_msghc_wait_result(struct vmbus_softc *,
	    struct vmbus_msghc *);
void	vmbus_msghc_wakeup(struct vmbus_softc *, const struct vmbus_message *);

#endif	/* !_VMBUS_VAR_H_ */