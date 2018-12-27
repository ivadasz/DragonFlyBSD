#include <machine/clock.h>
#include <machine/cputypes.h>

int cpu_vendor_id = CPU_VENDOR_INTEL;
int tsc_mpsync = 1;
int tsc_invariant = 1;
tsc_uclock_t tsc_frequency = 2400015357;
int tsc_present = 1;

/* XXX Add a SYSINIT call here */
