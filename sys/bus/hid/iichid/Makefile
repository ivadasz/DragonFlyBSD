SYSDIR?= ${.CURDIR}/../../..

KMOD= iichid

SRCS= iichid.c
SRCS+= bus_if.h device_if.h pci_if.h opt_acpi.h acpi_if.h hid_if.h

.include "../../../dev/acpica/Makefile.inc"

.include <bsd.kmod.mk>
