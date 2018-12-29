#include <machine/clock.h>
#include <machine/cputypes.h>
#include <machine/globaldata.h>
#include <machine/specialreg.h>

int cpu_vendor_id = CPU_VENDOR_INTEL;
int tsc_mpsync = 1;
int tsc_invariant = 1;
tsc_uclock_t tsc_frequency = 2400015357;
int tsc_present = 1;
int cpu_feature = CPUID_TSC;

/* XXX Add a SYSINIT call here to read all the sysctl. */

static struct privatespace prvspace_arr[1];
struct privatespace *CPU_prvspace = &prvspace_arr[0];

struct globaldata *
globaldata_find(int cpu)
{
	return &CPU_prvspace[cpu].mdglobaldata.mi;
}

