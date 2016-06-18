#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/runtime_pm.h>
#include <sys/taskqueue.h>
#include <sys/queue.h>

MALLOC_DEFINE(M_RPM, "m_rpm", "RPM memory allocations");

static int do_rpm = 0;
TUNABLE_INT("kern.do_runtime_suspend", &do_rpm);

struct rpm_client {
	SLIST_ENTRY(rpm_client) entries;
	struct rpm_ops *ops;
	device_t dev;
	struct callout to;
	struct task suspend_tsk;
	int state;	/* 0 == normal, 1 == runtime suspended */
	int target;
	int refcnt;
	int autosuspend_delay;
};

SLIST_HEAD(rpm_store, rpm_client) store;

static struct lock rpm_lock;

static void
runtime_pm_timeout(void *arg)
{
	struct rpm_client *client = arg;

	device_printf(client->dev, "%s: autosuspend timer triggered, refcnt=%d\n",
	    __func__, client->refcnt);
	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	if (client->state == 0 && client->refcnt <= 0)
		client->target = 1;
	if (client->state != client->target)
		taskqueue_enqueue(taskqueue_swi_mp, &client->suspend_tsk);
	lockmgr(&rpm_lock, LK_RELEASE);
}

static void
runtime_pm_suspend_task(void *arg, int pending)
{
	struct rpm_client *client = arg;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	if (client->target == 0 && client->refcnt == 0) {
		/* no need to wake up */
		client->target = 1;
	}
	if (client->target == 1 && client->refcnt > 0) {
		/* no need to suspend */
		client->target = 0;
	}
	if (client->target == client->state)
		goto done;

	struct timeval tv = {
		client->autosuspend_delay / 1000,
		(client->autosuspend_delay % 1000) * 1000
	};
	if (client->target == 0) {
		lockmgr(&rpm_lock, LK_RELEASE);
		kprintf("%s: Calling runtime resume\n", __func__);
		if (client->ops->runtime_resume(client->dev) != 0) {
			kprintf("%s: Failed to fresume from suspend state\n",
			    __func__);
			callout_reset(&client->to, tvtohz_high(&tv), runtime_pm_timeout, client);
			lockmgr(&rpm_lock, LK_EXCLUSIVE);
		} else {
			lockmgr(&rpm_lock, LK_EXCLUSIVE);
			client->state = 0;
		}
	} else if (client->target == 1) {
		lockmgr(&rpm_lock, LK_RELEASE);
		kprintf("%s: Calling runtime suspend\n", __func__);
		if (client->ops->runtime_suspend(client->dev) != 0) {
			kprintf("%s: Failed to suspend\n",
			    __func__);
			callout_reset(&client->to, tvtohz_high(&tv), runtime_pm_timeout, client);
			lockmgr(&rpm_lock, LK_EXCLUSIVE);
		} else {
			lockmgr(&rpm_lock, LK_EXCLUSIVE);
			client->state = 1;
		}
	}

done:
	lockmgr(&rpm_lock, LK_RELEASE);
	wakeup(client);
}

static struct rpm_client *
runtime_pm_lookup(device_t dev)
{
	struct rpm_client *client;

	client = NULL;
	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	SLIST_FOREACH(client, &store, entries) {
		if (client->dev == dev)
			break;
	}
	lockmgr(&rpm_lock, LK_RELEASE);

	return client;
}

void
pm_runtime_set_autosuspend_delay(device_t dev, int delay)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	device_printf(client->dev, "%s: Setting autosuspend delay to %dms\n",
	    __func__, delay);

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	client->autosuspend_delay = delay;
	lockmgr(&rpm_lock, LK_RELEASE);
}

void
pm_runtime_register(device_t dev, struct rpm_ops *ops)
{
	struct rpm_client *client;

	if (!do_rpm)
		return;

	device_printf(dev, "%s: Registering runtime pm device\n", __func__);

	client = kmalloc(sizeof(*client), M_RPM, M_WAITOK | M_ZERO);
	callout_init_mp(&client->to);
	TASK_INIT(&client->suspend_tsk, 0, runtime_pm_suspend_task, client);
	client->dev = dev;
	client->state = 0;
	client->refcnt = 0;
	client->ops = ops;
	client->autosuspend_delay=100000;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	SLIST_INSERT_HEAD(&store, client, entries);
	lockmgr(&rpm_lock, LK_RELEASE);
}

void
pm_runtime_unregister(device_t dev)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	SLIST_REMOVE(&store, client, rpm_client, entries);
	lockmgr(&rpm_lock, LK_RELEASE);
	callout_drain(&client->to);
	kfree(client, M_RPM);
}

void
pm_runtime_mark_last_busy(device_t dev)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
	KKASSERT(client->refcnt > 0);
	client->state = 0;
	struct timeval tv = {
		client->autosuspend_delay / 1000,
		(client->autosuspend_delay % 1000) * 1000
	};
	lockmgr(&rpm_lock, LK_RELEASE);
//	device_printf(client->dev, "%s: resetting autosuspend callout\n", __func__);
	callout_reset(&client->to, tvtohz_high(&tv), runtime_pm_timeout, client);
}

void
pm_runtime_put_autosuspend(device_t dev)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	struct timeval tv = {
		client->autosuspend_delay / 1000,
		(client->autosuspend_delay % 1000) * 1000
	};

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
//	device_printf(client->dev, "%s: refcnt before: %d\n",
//	    __func__, client->refcnt);
	KKASSERT(client->refcnt > 0);
	client->refcnt--;
	callout_reset(&client->to, tvtohz_high(&tv), runtime_pm_timeout, client);
	lockmgr(&rpm_lock, LK_RELEASE);
}

void
pm_runtime_get_sync(device_t dev)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
//	device_printf(client->dev, "%s: refcnt before: %d\n",
//	    __func__, client->refcnt);
	KKASSERT(client->refcnt >= 0);
	client->refcnt++;
	if (client->state == 1) {
		client->target = 0;
		taskqueue_enqueue(taskqueue_swi_mp, &client->suspend_tsk);
		while (client->state != 0)
			lksleep(client, &rpm_lock, 0, "rpmget", hz);
	}
	lockmgr(&rpm_lock, LK_RELEASE);
}

void
pm_runtime_get_noresume(device_t dev)
{
	struct rpm_client *client;

	client = runtime_pm_lookup(dev);
	if (client == NULL)
		return;

	lockmgr(&rpm_lock, LK_EXCLUSIVE);
//	device_printf(client->dev, "%s: refcnt before: %d\n",
//	    __func__, client->refcnt);
	KKASSERT(client->refcnt >= 0);
	client->refcnt++;
	lockmgr(&rpm_lock, LK_RELEASE);
}

static void
runtime_pm_init(void)
{
	SLIST_INIT(&store);
	lockinit(&rpm_lock, "rpm", 0, LK_CANRECURSE);
}

static void
runtime_pm_uninit(void)
{
}

SYSINIT(runtimepminit, SI_SUB_PRE_DRIVERS, SI_ORDER_ANY, runtime_pm_init, NULL);
SYSUNINIT(runtimepmuninit, SI_SUB_PRE_DRIVERS, SI_ORDER_ANY, runtime_pm_uninit, NULL);
