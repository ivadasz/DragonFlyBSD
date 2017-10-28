# $FreeBSD: src/usr.sbin/kbdcontrol/Makefile,v 1.3.14.2 2001/04/25 12:09:45 ru Exp $

PROG=	kbdcontrol
SRCS=	kbdcontrol.c
MAN=	kbdcontrol.1
CFLAGS+=-I${.CURDIR}
DPADD=	${LIBKBDMAP}
LDADD=	-lprivate_kbdmap

CFLAGS+=	-I${_SHLIBDIRPREFIX}/usr/include/priv
LDFLAGS+=	${PRIVATELIB_LDFLAGS}

.include <bsd.prog.mk>
