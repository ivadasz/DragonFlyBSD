#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systimer.h>

#include <machine/cpufunc.h>
#include <machine/cputypes.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>

#define HV_NANOSECONDS_PER_SEC	1000000000L

static sysclock_t hv_cputimer_count(void);
static void hv_cputimer_construct(struct cputimer *, sysclock_t);
static void hv_cputimer_register(void);

static struct cputimer hv_cputimer = {
	SLIST_ENTRY_INITIALIZER,
	"Hyper-V",
	CPUTIMER_PRI_HYPERV,
	CPUTIMER_HYPERV,
	hv_cputimer_count,
	cputimer_default_fromhz,
	cputimer_default_fromus,
	hv_cputimer_construct,
	cputimer_default_destruct,
	HV_NANOSECONDS_PER_SEC / 100,
	0, 0, 0
};

static sysclock_t
hv_cputimer_count(void)
{
	u_int now = rdmsr(HV_X64_MSR_TIME_REF_COUNT);
	return (now);
}

static void
hv_cputimer_construct(struct cputimer *timer, sysclock_t oldclock)
{
	timer->base = 0;
	timer->base = oldclock - hv_cputimer_count();
}

static void
hv_cputimer_register(void)
{
	if (vmm_vendor_id == VMM_VENDOR_MICROSOFT &&
	    (hyperv_feature & HV_CPUID_HAS_TIMEREF)) {
		cputimer_register(&hv_cputimer);
		cputimer_select(&hv_cputimer, 0);
	}
}

SYSINIT(hv_cputimer_reg, SI_BOOT2_POST_SMP, SI_ORDER_FIRST,
	hv_cputimer_register, NULL);
