SYSDIR?= ${.CURDIR}/../../..

KMOD=	iichid
SRCS=	iichid.c
SRCS+=	device_if.h bus_if.h opt_acpi.h acpi_if.h smbus_if.h

.include "../../acpica/Makefile.inc"

.include <bsd.kmod.mk>
