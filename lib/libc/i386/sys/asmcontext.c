/* $DragonFly: src/lib/libc/i386/sys/asmcontext.c,v 1.1 2007/01/17 02:17:36 corecode Exp $ */

#define _KERNEL_STRUCTURES
#include <sys/types.h>
#include <sys/ucontext.h>
#include <sys/assym.h>
#include <machine/frame.h>
#include <machine/tss.h>
#include <machine/segments.h>
#include <stddef.h>

ASSYM(UC_SIGMASK, offsetof(ucontext_t, uc_sigmask));
ASSYM(UC_LINK, offsetof(ucontext_t, uc_link));
ASSYM(UC_MCONTEXT, offsetof(ucontext_t, uc_mcontext));
ASSYM(SIG_BLOCK, SIG_BLOCK);
ASSYM(SIG_SETMASK, SIG_SETMASK);
ASSYM(_MC_FPOWNED_NONE, _MC_FPOWNED_NONE);
ASSYM(_MC_FPFMT_NODEV, _MC_FPFMT_NODEV);
ASSYM(_MC_FPFMT_387, _MC_FPFMT_387);
ASSYM(_MC_FPFMT_XMM, _MC_FPFMT_XMM);
ASSYM(MC_LEN, offsetof(mcontext_t, mc_len));
ASSYM(SIZEOF_MCONTEXT_T, sizeof(mcontext_t));
ASSYM(SIZEOF_UCONTEXT_T, sizeof(ucontext_t));
