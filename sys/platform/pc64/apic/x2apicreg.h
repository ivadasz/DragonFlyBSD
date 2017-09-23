/*
 * Copyright (c) 2015 The DragonFly Project.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _MACHINE_X2APICREG_H_
#define _MACHINE_X2APICREG_H_

#define MSR_X2APIC_BASE	0x00000800

#define X2APIC_OFF_APICID	0x002
#define X2APIC_OFF_VERSION	0x003
#define X2APIC_OFF_TPR		0x008
#define X2APIC_OFF_EOI		0x00b
#define X2APIC_OFF_SVR		0x00f
#define X2APIC_OFF_ICR		0x030
#define X2APIC_OFF_TIMER	0x032
#define X2APIC_OFF_PCINT	0x034
#define X2APIC_OFF_LINT0	0x035
#define X2APIC_OFF_LINT1	0x036
#define X2APIC_OFF_LVT_ERROR	0x037
#define X2APIC_OFF_TIMER_ICR	0x038
#define X2APIC_OFF_CCR		0x039
#define X2APIC_OFF_DCR		0x03e

#endif /* _MACHINE_X2APICREG_H_ */
