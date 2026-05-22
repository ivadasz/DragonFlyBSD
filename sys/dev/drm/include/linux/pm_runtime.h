/*
 * Copyright (c) 2017-2019 François Tigeot <ftigeot@wolfpond.org>
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

#ifndef LINUX_PM_RUNTIME_H
#define LINUX_PM_RUNTIME_H

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/jiffies.h>

TASKQUEUE_DECLARE(pm_runtime);

enum rpm_status {
	RPM_SUSPENDED = 0,
	RPM_SUSPENDING,
	RPM_RESUMING,
	RPM_ACTIVE,
};

int pm_request_autosuspend(struct device *);
void pm_runtime_autosuspend(struct device *);
void pm_runtime_enable(struct device *);
void pm_runtime_disable(struct device *);
void pm_runtime_forbid(struct device *);
void pm_runtime_allow(struct device *);
void pm_runtime_init(struct device *);
void pm_runtime_use_autosuspend(struct device *);
void pm_runtime_dont_use_autosuspend(struct device *);
void pm_runtime_set_active(struct device *);
int pm_runtime_get_if_in_use(struct device *);
int pm_runtime_resume_sync(struct device *);
void pm_runtime_set_autosuspend_delay(struct device *, int);

static inline void
pm_runtime_mark_last_busy(struct device *dev)
{
	dev->power.last_busy = ticks;
}

static inline int
pm_request_idle(struct device *dev)
{
	kprintf("%s: Requesting pm_runtime_idle()\n", __func__);
	taskqueue_enqueue(taskqueue_pm_runtime, &dev->power.idle_task);
	return 0;
}

static inline void
pm_request_resume(struct device *dev)
{
	kprintf("%s: Requesting pm_runtime_resume()\n", __func__);
	taskqueue_enqueue(taskqueue_pm_runtime, &dev->power.resume_task);
}

static inline void
pm_runtime_get(struct device *dev)
{
	if (atomic_fetchadd_int(&dev->power.usage_count, 1) == 0)
		pm_request_resume(dev);
}

static inline void
pm_runtime_get_noresume(struct device *dev)
{
	atomic_add_acq_int(&dev->power.usage_count, 1);
}

static inline int
pm_runtime_get_sync(struct device *dev)
{
	atomic_add_acq_int(&dev->power.usage_count, 1);
	if (atomic_load_int(&dev->power.runtime_status) == RPM_ACTIVE)
		return 0;
	return pm_runtime_resume_sync(dev);
}

static inline void
pm_runtime_put(struct device *dev)
{
	if (atomic_fetchadd_int(&dev->power.usage_count, -1) == 1)
		pm_request_idle(dev);
}

static inline void
pm_runtime_put_autosuspend(struct device *dev)
{
	dev->power.last_busy = ticks;
	if (atomic_fetchadd_int(&dev->power.usage_count, -1) == 1)
		pm_request_autosuspend(dev);
}

#endif	/* LINUX_PM_RUNTIME_H */
