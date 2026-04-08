/*	$OpenBSD: s_logbf.S,v 1.4 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(logbf)
	flds	4(%esp)
	fxtract
	fstp	%st
	ret
#if 0
END_STD(logbf)
#else
END(logbf)
#endif

.section .note.GNU-stack,"",%progbits
