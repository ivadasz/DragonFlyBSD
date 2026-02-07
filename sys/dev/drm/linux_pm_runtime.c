/*
 * Copyright (c) 2026 Imre Vadász <imre@vdsz.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/jiffies.h>

static int __pm_runtime_autosuspend_locked(struct device *dev);
static int __pm_runtime_resume_locked(struct device *dev);
static int __pm_runtime_resume_sync_locked(struct device *dev);
static int __pm_runtime_idle_locked(struct device *dev);
static int __pm_request_autosuspend_locked(struct device *dev);

static int
__pm_runtime_resume_locked(struct device *dev)
{
	kprintf("%s: Running resume\n", __func__);

	BUG_ON(dev->type == NULL);
	if (dev->power.disable_depth != 0)
		return -EACCES;
	if (dev->power.runtime_status == RPM_ACTIVE)
		return 1;
	if (dev->type != NULL) {
		lockmgr(&dev->power.lock, LK_RELEASE);
		int ret = dev->type->pm->runtime_resume(dev);
		lockmgr(&dev->power.lock, LK_EXCLUSIVE);
		if (ret >= 0) {
			dev->power.last_error = 0;
			dev->power.is_suspended = 0;
			dev->power.runtime_status = RPM_ACTIVE;
		} else {
			dev->power.last_error = ret;
			dev->power.is_suspended = 1;
			dev->power.runtime_status = RPM_SUSPENDED;
		}
		return ret;
	}
	return 0;
}

static int
pm_runtime_suspend(struct device *dev)
{
	int ret = 0;

	kprintf("%s: Running suspend\n", __func__);
	BUG_ON(dev->type == NULL);
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	if (dev->power.disable_depth != 0) {
		ret = -EACCES;
		goto done;
	}
	if (dev->power.runtime_status == RPM_SUSPENDED) {
		ret = 1;
		goto done;
	}
	if (atomic_load_int(&dev->power.usage_count) > 0) {
		ret = -EBUSY;
		dev->power.runtime_status = RPM_ACTIVE;
		dev->power.is_suspended = 0;
		goto done;
	}
	if (dev->type != NULL) {
		/*
		 * This prevents races against threads that synchronously
		 * get a runtime pm reference (i.e. pm_runtime_get_sync()).
		 */
		atomic_store_rel_int(&dev->power.runtime_status, RPM_SUSPENDING);
		if (atomic_load_acq_int(&dev->power.usage_count) > 0) {
			ret = -EBUSY;
			dev->power.runtime_status = RPM_ACTIVE;
			dev->power.is_suspended = 0;
			goto done;
		}
		lockmgr(&dev->power.lock, LK_RELEASE);
		ret = dev->type->pm->runtime_suspend(dev);
		lockmgr(&dev->power.lock, LK_EXCLUSIVE);
		if (ret >= 0) {
			dev->power.is_suspended = 1;
			dev->power.runtime_status = RPM_SUSPENDED;
			dev->power.last_error = 0;
		} else {
			dev->power.is_suspended = 0;
			dev->power.runtime_status = RPM_ACTIVE;
			dev->power.last_error = ret;
		}
	}
done:
	lockmgr(&dev->power.lock, LK_RELEASE);
	return ret;
}

static int
__pm_request_autosuspend_locked(struct device *dev)
{
	kprintf("%s: Requesting autosuspend use_autosuspend=%d autosuspend_delay=%d runtime_status=%d\n",
	    __func__, dev->power.use_autosuspend, dev->power.autosuspend_delay,
	    dev->power.runtime_status);
	if (dev->power.use_autosuspend && dev->power.autosuspend_delay > 0 &&
	    dev->power.runtime_status == RPM_ACTIVE) {
		kprintf("%s: Enqueueing autosuspend\n", __func__);
		taskqueue_enqueue_timeout(taskqueue_thread[0],
		    &dev->power.suspend_task,
		    dev->power.autosuspend_delay/(1000/hz));
	}
	return 0;
}

int
pm_request_autosuspend(struct device *dev)
{
	int ret;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	ret = __pm_request_autosuspend_locked(dev);
	lockmgr(&dev->power.lock, LK_RELEASE);
	return ret;
}

static int
__pm_runtime_autosuspend_locked(struct device *dev)
{
	int ret = 0;

	BUG_ON(dev->type == NULL);
	pm_runtime_mark_last_busy(dev);
	if (dev->power.disable_depth != 0) {
		return -EACCES;
	}
	if (dev->power.runtime_status == RPM_SUSPENDED) {
		return 1;
	}
	if (dev->type != NULL && dev->power.use_autosuspend)
		__pm_request_autosuspend_locked(dev);
	return ret;
}

void
pm_runtime_autosuspend(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	__pm_runtime_autosuspend_locked(dev);
	lockmgr(&dev->power.lock, LK_RELEASE);
}

static int
__pm_runtime_idle_locked(struct device *dev)
{
	int ret = 0;

	BUG_ON(dev->type == NULL);
	if (dev->power.disable_depth != 0)
		return -EACCES;
	if (dev->type != NULL) {
		if (dev->power.idle_running)
			return -EINPROGRESS;

		if (dev->type->pm->runtime_idle != NULL) {
			dev->power.idle_running = 1;
			lockmgr(&dev->power.lock, LK_RELEASE);
			ret = dev->type->pm->runtime_idle(dev);
			lockmgr(&dev->power.lock, LK_EXCLUSIVE);
			if (ret > 0)
				dev->power.last_error = 0;
			else
				dev->power.last_error = ret;
			dev->power.idle_running = 0;
		}

		if (ret == 0)
			return __pm_runtime_autosuspend_locked(dev);
	}
	return ret;
}

void
pm_runtime_enable(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	BUG_ON(dev->power.disable_depth == 0);
	dev->power.disable_depth--;
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_disable(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	dev->power.disable_depth++;
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_set_active(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	dev->power.last_error = 0;
	dev->power.runtime_status = RPM_ACTIVE;
	dev->power.is_suspended = 0;
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_forbid(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	dev->power.runtime_auto = 0;
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_allow(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	dev->power.runtime_auto = 1;
	lockmgr(&dev->power.lock, LK_RELEASE);
}

static int
__pm_runtime_resume_sync_locked(struct device *dev)
{
	if (dev->power.runtime_status == RPM_SUSPENDING) {
		kprintf("%s: Waiting for suspend to finish\n", __func__);
		lockmgr(&dev->power.lock, LK_RELEASE);
		taskqueue_drain_timeout(taskqueue_thread[0],
		    &dev->power.suspend_task);
		lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	}
	if (dev->power.runtime_status == RPM_SUSPENDED) {
		kprintf("%s: Enqueueing resume\n", __func__);
		print_backtrace(-1);
		dev->power.runtime_status = RPM_RESUMING;
		taskqueue_enqueue(taskqueue_thread[0], &dev->power.resume_task);
	}
	if (dev->power.runtime_status == RPM_RESUMING) {
		lockmgr(&dev->power.lock, LK_RELEASE);
		taskqueue_drain(taskqueue_thread[0], &dev->power.resume_task);
		lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	}
	if (dev->power.runtime_status == RPM_ACTIVE) {
		return 0;
	} else {
		return dev->power.last_error ? dev->power.last_error : -EINVAL;
	}
}

static int
__pm_runtime_get_sync_locked(struct device *dev)
{
	atomic_add_acq_int(&dev->power.usage_count, 1);
	return __pm_runtime_resume_sync_locked(dev);
}

int
pm_runtime_resume_sync(struct device *dev)
{
	int ret;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	ret = __pm_runtime_resume_sync_locked(dev);
	lockmgr(&dev->power.lock, LK_RELEASE);

	return ret;
}

int
pm_runtime_get_if_in_use(struct device *dev)
{
	int ret = 0;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	if (dev->power.disable_depth != 0) {
		ret = -EINVAL;
		goto done;
	}
	if (dev->power.runtime_status == RPM_ACTIVE) {
		if (atomic_fetchadd_int(&dev->power.usage_count, 1) > 0)
			ret = 1;
	}
done:
	if (ret <= 0) {
		kprintf("%s: runtime status not RPM_ACTIVE ret=%d status=%d usage_count=%d\n",
		    __func__, ret, dev->power.runtime_status,
		    atomic_load_int(&dev->power.usage_count));
	}
	lockmgr(&dev->power.lock, LK_RELEASE);
	return ret;
}

void
pm_runtime_set_autosuspend_delay(struct device *dev, int autosuspend_delay)
{
	int old;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	old = dev->power.autosuspend_delay;
	dev->power.autosuspend_delay = autosuspend_delay;
	if (dev->power.use_autosuspend) {
		if (old < 0 && autosuspend_delay >= 0) {
			/* Dropping reference from disabling autosuspend */
			atomic_subtract_int(&dev->power.usage_count, 1);
		} else if (old > 0 && autosuspend_delay < 0) {
			/* Disabling autosuspend so we should get a reference */
			__pm_runtime_get_sync_locked(dev);
		}
	} else {
		__pm_runtime_idle_locked(dev);
	}
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_use_autosuspend(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	if (dev->power.use_autosuspend == 0 &&
	    dev->power.autosuspend_delay >= 0) {
		dev->power.use_autosuspend = 1;
		__pm_runtime_get_sync_locked(dev);
		lockmgr(&dev->power.lock, LK_RELEASE);
	} else {
		lockmgr(&dev->power.lock, LK_RELEASE);
	}
}

void
pm_runtime_dont_use_autosuspend(struct device *dev)
{
	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	if (dev->power.use_autosuspend == 1 &&
	    dev->power.autosuspend_delay < 0) {
		dev->power.use_autosuspend = 0;
		lockmgr(&dev->power.lock, LK_RELEASE);
		if (atomic_fetchadd_int(&dev->power.usage_count, -1) == 1)
			__pm_runtime_idle_locked(dev);
	} else {
		lockmgr(&dev->power.lock, LK_RELEASE);
	}
}

static void
run_suspend(void *context, int pending)
{
	struct device *dev = context;

	pm_runtime_suspend(dev);
}

static void
run_resume(void *context, int pending)
{
	struct device *dev = context;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	__pm_runtime_resume_locked(dev);
	lockmgr(&dev->power.lock, LK_RELEASE);
}

static void
run_idle(void *context, int pending)
{
	struct device *dev = context;

	lockmgr(&dev->power.lock, LK_EXCLUSIVE);
	__pm_runtime_idle_locked(dev);
	lockmgr(&dev->power.lock, LK_RELEASE);
}

void
pm_runtime_init(struct device *dev)
{
	dev->power.disable_depth = 1;
	dev->power.usage_count = 0;
	dev->power.runtime_auto = 1;
	dev->power.runtime_status = RPM_SUSPENDED;
	dev->power.idle_running = 0;
	dev->power.is_suspended = 1;
	dev->power.last_error = 0;
	dev->power.autosuspend_delay = 0;
	dev->power.last_busy = ticks;
	dev->power.timer_expires = 0;
	dev->power.use_autosuspend = 0;
	lockinit(&dev->power.lock, "rpm", 0, 0);
	TIMEOUT_TASK_INIT(taskqueue_thread[0], &dev->power.suspend_task, 0,
	    run_suspend, dev);
	TASK_INIT(&dev->power.resume_task, 0, run_resume, dev);
	TASK_INIT(&dev->power.idle_task, 0, run_idle, dev);
}
