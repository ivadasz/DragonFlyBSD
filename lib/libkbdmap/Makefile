LIB=	private_kbdmap${LIB_SUFFIX}
PRIVATELIB=
SHLIB_MAJOR= 0
INCSDIR=	${INCLUDEDIR}/priv

SHLIBDIR?=      /lib

WARNS?=	6

SRCDIR=	${.CURDIR}
CFLAGS+=-I${.CURDIR}

DPADD=	${LIBL}
LDADD=	-ll

SRCS=	kbdmap.c lex.l
INCS=	kbdmap.h

MAN=	kbdmap.3 kbdmap.5
MLINKS= kbdmap.5 keymap.5

.include <bsd.lib.mk>
