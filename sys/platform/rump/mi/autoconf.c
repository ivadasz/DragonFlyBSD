/*-
 * Copyright (c) 1990 The Regents of the University of California.
 * Copyright (c) 2008 The DragonFly Project.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * William Jolitz.
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
 *	from: @(#)autoconf.c	7.1 (Berkeley) 5/9/91
 * $FreeBSD: src/sys/i386/i386/autoconf.c,v 1.146.2.2 2001/06/07 06:05:58 dd Exp $
 */

/*
 * Setup the system to run on the current machine.
 *
 * Configure() is called at boot time and initializes the vba
 * device tables and the memory controller monitoring.  Available
 * devices are determined (from possibilities mentioned in ioconf.c),
 * and the drivers are initialized.
 */
#include "opt_rootdevname.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bootmaj.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/diskslice.h>
#include <sys/reboot.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/cons.h>
#include <sys/thread.h>
#include <sys/device.h>

#include <machine/smp.h>
#include <machine/globaldata.h>
#include <machine/md_var.h>

#include <vm/vm_extern.h>

static void cpu_startup (void *);
static void configure_first (void *);
static void configure (void *);
static void configure_final (void *);

SYSINIT(cpu, SI_BOOT2_START_CPU, SI_ORDER_FIRST, cpu_startup, NULL);
SYSINIT(configure1, SI_SUB_CONFIGURE, SI_ORDER_FIRST, configure_first, NULL);
/* SI_ORDER_SECOND is hookable */
SYSINIT(configure2, SI_SUB_CONFIGURE, SI_ORDER_THIRD, configure, NULL);
/* SI_ORDER_MIDDLE is hookable */
SYSINIT(configure3, SI_SUB_CONFIGURE, SI_ORDER_ANY, configure_final, NULL);

cdev_t	rootdev = NULL;

/*
 *
 */
static void
cpu_startup(void *dummy)
{
	kprintf("%s", version);

	if (nbuf == 0) {
		int factor = 4 * NBUFCALCSIZE / 1024;
		/* XXX Fix this calculation */
		int kbytes = 128*1024*1024 * (PAGE_SIZE / 1024);

		nbuf = 50;
		if (kbytes > 4096)
			nbuf += min((kbytes - 4096) / factor, 65536 / factor);
		if (kbytes > 65536)
			nbuf += (kbytes - 65536) * 2 / (factor * 5);
#ifndef _RUMPKERNEL
		if (maxbcache && nbuf > maxbcache / NBUFCALCSIZE)
			nbuf = maxbcache / NBUFCALCSIZE;
#endif
	}

	/*
	 * Allocate memory for the buffer cache
	 */
	buf = (void *)kmem_alloc(&kernel_map,
				 nbuf * sizeof(struct buf),
				 VM_SUBSYS_BUF);
	swbuf_mem = (void *)kmem_alloc(&kernel_map,
				       nswbuf_mem * sizeof(struct buf),
				       VM_SUBSYS_BUF);
	swbuf_kva = (void *)kmem_alloc(&kernel_map,
				       nswbuf_kva * sizeof(struct buf),
				       VM_SUBSYS_BUF);

#if 0
	mp_start();
	mp_announce();
	cpu_setregs();
#endif
}

/*
 * Determine i/o configuration for a machine.
 */
static void
configure_first(void *dummy)
{
}

static void
configure(void *dummy)
{
#if 0
        /*
	 * Final interrupt support acviation, then enable hardware interrupts.
	 */
	MachIntrABI.finalize();
	cpu_enable_intr();

	/*
	 * This will configure all devices, generally starting with the
	 * nexus (pc64/x86_64/nexus.c).  The nexus ISA code explicitly
	 * dummies up the attach in order to delay legacy initialization
	 * until after all other busses/subsystems have had a chance
	 * at those resources.
	 */
	root_bus_configure();

	/*
	 * Allow lowering of the ipl to the lowest kernel level if we
	 * panic (or call tsleep() before clearing `cold').  No level is
	 * completely safe (since a panic may occur in a critical region
	 * at splhigh()), but we want at least bio interrupts to work.
	 */
	safepri = TDPRI_KERN_USER;
#endif
}

/*
 * Finalize configure.  Reprobe for the console, in case it was one
 * of the devices which attached, then finish console initialization.
 */
static void
configure_final(void *dummy)
{
	cninit();
	cninit_finish();

	if (bootverbose)
		kprintf("Device configuration finished.\n");
}
