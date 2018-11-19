#include <sys/types.h>
#include <bus/isa/isareg.h>
#include <sys/bus_private.h>

struct config_resource sio0_resources[] = {
	{ "at",	RES_STRING,	{ (long)"isa" }},
	{ "flags",	RES_INT,	{ 0x10 }},
	{ "port",	RES_INT,	 { IO_COM1 }},
	{ "irq",	RES_INT,	{ 4 }},
};
#define sio0_count 4
struct config_device config_devtab[] = {
	{ "sio",	0,	sio0_count,	sio0_resources },
};
int devtab_count = 1;
