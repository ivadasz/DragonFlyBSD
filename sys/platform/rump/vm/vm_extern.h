/*-
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
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
 *	@(#)vm_extern.h	8.2 (Berkeley) 1/12/94
 * $FreeBSD: src/sys/vm/vm_extern.h,v 1.46.2.3 2003/01/13 22:51:17 dillon Exp $
 */

#ifndef _VM_VM_EXTERN_H_
#define	_VM_VM_EXTERN_H_

#ifndef _SYS_TYPES_H_
#include <sys/types.h>
#endif
#ifndef _MACHINE_TYPES_H_
#include <machine/types.h>
#endif

#ifdef _KERNEL

#define KM_CPU(i) 0
#define VM_SUBSYS_GD 0
#define VM_SUBSYS_BOGUS 1

typedef void *vm_map_t;
typedef int vm_subsys_t;

struct kmem_anon_desc {
	vm_offset_t off;
	vm_size_t size;
};

extern int kernel_map;

typedef struct kmem_anon_desc kmem_anon_desc_t;

vm_offset_t kmem_alloc (vm_map_t map, vm_size_t size, vm_subsys_t id);
void kmem_free (vm_map_t map, vm_offset_t off, vm_size_t size);

static __inline
vm_offset_t
kmem_alloc_stack (vm_map_t map, vm_size_t size, int kmflags)
{
	return kmem_alloc (map, size, 0);
}

static __inline
vm_offset_t
kmem_alloc3 (vm_map_t map, vm_size_t size, vm_subsys_t id, int flags)
{
	return kmem_alloc (map, size, id);
}

static __inline
vm_offset_t
kmem_alloc_pageable (vm_map_t map, vm_size_t size, vm_subsys_t id)
{
	return kmem_alloc (map, size, id);
}

static __inline
void *
kmem_alloc_swapbacked(kmem_anon_desc_t *kp, vm_size_t size, vm_subsys_t id)
{
	vm_offset_t off = kmem_alloc (NULL, size, 0);
	kp->off = off;
	kp->size = size;
}

static __inline
void
kmem_free_swapbacked(kmem_anon_desc_t *kp)
{
	kmem_free(NULL, kp->off, kp->size);
}


#endif				/* _KERNEL */

#endif				/* !_VM_VM_EXTERN_H_ */
