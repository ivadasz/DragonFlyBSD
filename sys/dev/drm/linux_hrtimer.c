/*
 * Copyright (c) 2017 François Tigeot
 * Copyright (c) 2017 Imre Vadász
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

#include <linux/hrtimer.h>
#include <linux/bug.h>

#include <sys/systimer.h>
#include <sys/lock.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

struct shared_hrtimer;

static void common_hrtimer_init(void *arg);
static void common_hrtimer_function(void *context, int prio __unused);
static void common_hrtimer_handler(systimer_t info, int in_ipi __unused,
    struct intrframe *frame __unused);
static void common_hrtimer_oneshot(struct shared_hrtimer *data,
    struct timeval *expire);
static void common_hrtimer_cancel(struct shared_hrtimer *data);
static void common_hrtimer_insertnext(struct shared_hrtimer *common,
    struct hrtimer *timer);
static void common_hrtimer_enqueue(struct shared_hrtimer *common,
    struct hrtimer *timer);
static int common_hrtimer_dequeue(struct shared_hrtimer *common,
    struct hrtimer *timer);
static void __hrtimer_function(void *arg);

TAILQ_HEAD(hrtimer_timeouts, hrtimer);

/*
 * Handle all hrtimer interrupts on a single cpu. The cpu is chosen by the
 * first hrtimer that is scheduled when the timer is disabled.
 */
static struct shared_hrtimer {
	struct systimer *timer;
	struct lock timer_lock;
	struct timeval target;	/* valid if timer != NULL && timer->data != NULL */
	struct task hrtimer_task;
	bool infunction;

	/* Current batch of timeouts. Doesn't need to be sorted. */
	struct hrtimer_timeouts hrtimer_activeq;
	/* Next batch of timeouts. Needs to be sorted by timeout value. */
	struct hrtimer_timeouts hrtimer_nextq;
} common_hrtimer;

SYSINIT(hrtimer_setup, SI_SUB_DRIVERS, SI_ORDER_FIRST, common_hrtimer_init,
	NULL);

static void
common_hrtimer_init(void *arg)
{
	common_hrtimer.timer =
	    kmalloc(sizeof(struct systimer), M_DEVBUF, M_NOWAIT | M_ZERO);
	lockinit(&common_hrtimer.timer_lock, "hrtimer", 0, LK_CANRECURSE);
	TAILQ_INIT(&common_hrtimer.hrtimer_activeq);
	TAILQ_INIT(&common_hrtimer.hrtimer_nextq);
	TASK_INIT(&common_hrtimer.hrtimer_task, 0, common_hrtimer_function,
	    &common_hrtimer);
}

/* XXX uninit code for common_hrtimer */

static void
common_hrtimer_handler(systimer_t info, int in_ipi __unused,
    struct intrframe *frame __unused)
{
	struct shared_hrtimer *data = info->data;

	taskqueue_enqueue(taskqueue_thread[mycpuid], &data->hrtimer_task);
}

/* This is supposed to run in taskqueue context. */
static void
common_hrtimer_function(void *context, int prio __unused)
{
	struct shared_hrtimer *data = context;
	struct hrtimer *t, *temp;
	struct timeval now, next_expire, next_target;

	lockmgr(&data->timer_lock, LK_EXCLUSIVE);
	data->infunction = true;
	TAILQ_FOREACH_MUTABLE(t, &data->hrtimer_activeq, entries, temp) {
		TAILQ_REMOVE(&data->hrtimer_activeq, t, entries);
		__hrtimer_function(t);
	}
	data->timer->data = NULL;
	if (TAILQ_EMPTY(&data->hrtimer_nextq)) {
		data->infunction = false;
		goto done;
	}
	microuptime(&now);
	while (true) {
		bool didwork = false;
		/* Check if any more entries timed out already. */
		TAILQ_FOREACH(t, &data->hrtimer_nextq, entries) {
			if (timevalcmp(&now, &t->expire, >=)) {
				didwork = true;
				TAILQ_REMOVE(&data->hrtimer_nextq, t, entries);
				__hrtimer_function(t);
				microuptime(&now);
			}
		}
		if (!didwork)
			break;
	}
	data->infunction = false;
	if (TAILQ_EMPTY(&data->hrtimer_nextq)) {
		goto done;
	}
	temp = TAILQ_FIRST(&data->hrtimer_nextq);
	next_expire = temp->expire;
	KKASSERT(timevalcmp(&next_expire, &now, >));

	/* Compute expiry time for computing batching. */
	next_target = (struct timeval){0, temp->slackus};
	timevaladd(&next_target, &next_expire);
	data->target = next_target;

	timevalsub(&next_expire, &now);
	TAILQ_REMOVE(&data->hrtimer_nextq, temp, entries);
	TAILQ_INSERT_TAIL(&data->hrtimer_activeq, temp, entries);
	TAILQ_FOREACH(t, &data->hrtimer_nextq, entries) {
		if (timevalcmp(&t->expire, &next_target, <=)) {
			TAILQ_REMOVE(&data->hrtimer_nextq, t, entries);
			TAILQ_INSERT_TAIL(&data->hrtimer_activeq, t, entries);
		}
	}
	if (!TAILQ_EMPTY(&data->hrtimer_activeq)) {
		common_hrtimer_oneshot(data, &next_expire);
	}
done:
	lockmgr(&data->timer_lock, LK_RELEASE);
}

static void
common_hrtimer_oneshot(struct shared_hrtimer *data, struct timeval *expire)
{
	int us = expire->tv_usec;

	if (data->timer == NULL) {
		data->timer = kmalloc(sizeof(struct systimer), M_DEVBUF,
		    M_INTWAIT | M_ZERO);
	}
	systimer_init_oneshot(data->timer, common_hrtimer_handler, data, us);
}

/* Needs timer_lock */
static void
common_hrtimer_cancel(struct shared_hrtimer *data)
{
	lockmgr(&data->timer_lock, LK_EXCLUSIVE);
	/*
	 * XXX Untangle the systimer_t pointer stuff, to allow asynchronously
	 *     stopping a oneshot timer running on a different cpu core.
	 */
	if (data->timer->gd == mycpu) {
		systimer_del(data->timer);
		data->timer->data = NULL;
		while (taskqueue_cancel(taskqueue_thread[mycpuid],
		    &data->hrtimer_task, NULL) != 0) {
			taskqueue_drain(taskqueue_thread[mycpuid],
			    &data->hrtimer_task);
		}
	}
	lockmgr(&data->timer_lock, LK_RELEASE);
}

static void
common_hrtimer_insertnext(struct shared_hrtimer *common, struct hrtimer *timer)
{
	struct hrtimer *t = NULL;

	/* Insert into sorted common->hrtimer_nextq */
	TAILQ_FOREACH(t, &common->hrtimer_nextq, entries) {
		if (timevalcmp(&t->expire, &timer->expire, >))
			break;
	}
	if (t == NULL) {
		TAILQ_INSERT_TAIL(&common->hrtimer_activeq, timer, entries);
	} else {
		TAILQ_INSERT_BEFORE(t, timer, entries);
	}
}

static void
common_hrtimer_enqueue(struct shared_hrtimer *common, struct hrtimer *timer)
{
	lockmgr(&common->timer_lock, LK_EXCLUSIVE);
	if (common->timer != NULL && common->timer->data != NULL) {
		if (!common->infunction &&
		    timevalcmp(&timer->expire, &common->target, <)) {
			/* XXX Sorting shouldn't be needed here. */
			TAILQ_INSERT_TAIL(&common->hrtimer_activeq, timer,
			    entries);
		} else {
			common_hrtimer_insertnext(common, timer);
		}
	} else {
		struct timeval next_expire, next_target;

		KKASSERT(TAILQ_EMPTY(&common->hrtimer_nextq));

		/* Directly go to active queue, and schedule timer. */
		TAILQ_INSERT_TAIL(&common->hrtimer_activeq, timer, entries);

		/* Compute expiry time for computing batching. */
		next_target = (struct timeval){0, timer->slackus};
		timevaladd(&next_target, &timer->expire);
		common->target = next_target;

		next_expire = timer->expire;
		timevalsub(&next_expire, &timer->now_inserted);
		common_hrtimer_oneshot(common, &next_expire);
	}
	lockmgr(&common->timer_lock, LK_RELEASE);
}

static int
common_hrtimer_dequeue(struct shared_hrtimer *common, struct hrtimer *timer)
{
	struct hrtimer *t = NULL;
	int ret = 0;

	if (!timer->active)
		return 0;

	lockmgr(&common->timer_lock, LK_EXCLUSIVE);
	TAILQ_FOREACH(t, &common->hrtimer_activeq, entries) {
		if (t == timer)
			break;
	}
	if (t != NULL) {
		TAILQ_REMOVE(&common->hrtimer_activeq, t, entries);
		ret = 1;
		if (TAILQ_EMPTY(&common->hrtimer_activeq) &&
		    !common->infunction) {
			common_hrtimer_cancel(common);
		}
	} else {
		TAILQ_FOREACH(t, &common->hrtimer_nextq, entries) {
			if (t == timer)
				break;
		}
		if (t != NULL) {
			TAILQ_REMOVE(&common->hrtimer_nextq, t, entries);
			ret = 1;
		}
	}
	lockmgr(&common->timer_lock, LK_RELEASE);
	return ret;
}

static void
__hrtimer_function(void *arg)
{
	struct hrtimer *timer = arg;
	enum hrtimer_restart restart = HRTIMER_RESTART;

	if (timer->function) {
		restart = timer->function(timer);
	}

	if (restart == HRTIMER_RESTART) {
		/*
		 * XXX Not implemented yet, we would need to store the
		 *     expiration period, to do the callout_reset here.
		 */
	} else {
		timer->active = false;
	}
}

void
hrtimer_init(struct hrtimer *timer, clockid_t clock_id,
			   enum hrtimer_mode mode)
{
	BUG_ON(clock_id != CLOCK_MONOTONIC);

	memset(timer, 0, sizeof(struct hrtimer));
	timer->clock_id = clock_id;
	timer->ht_mode = mode;

	lwkt_token_init(&timer->timer_token, "timer token");
}

/* XXX Make sure that timeouts >1s are forbidden. */
void
hrtimer_start_range_ns(struct hrtimer *timer, ktime_t tim,
		       u64 range_ns, const enum hrtimer_mode mode)
{
	struct timeval target, tmp;
	uint64_t to;

	KKASSERT(tim.tv64 > 0);
	to = tim.tv64;
	KKASSERT(range_ns + to + 999 < 1000 * 1000 * 1000);

	tmp = (struct timeval){0, (to + 999) / 1000};
	microuptime(&timer->now_inserted);
	target = timer->now_inserted;
	timevaladd(&target, &tmp);
	timer->expire = target;
	timer->slackus = (range_ns + 999) / 1000;

	lwkt_gettoken(&timer->timer_token);

	timer->active = true;
	common_hrtimer_enqueue(&common_hrtimer, timer);

	lwkt_reltoken(&timer->timer_token);
}

int
hrtimer_cancel(struct hrtimer *timer)
{
	if (timer->active) {
		if (common_hrtimer_dequeue(&common_hrtimer, timer))  {
			timer->active = 0;
			return 1;
		}
	}
	return 0;
}

/* Returns non-zero if the timer is already on the queue */
bool
hrtimer_active(const struct hrtimer *timer)
{

	return timer->active;
}
