/*
 * Copyright (c) 2005, David Xu <davidxu@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
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
 *
 */

#include "namespace.h"
#include <sys/signalvar.h>
#include <machine/tls.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "un-namespace.h"

#include "thr_private.h"

/* #define DEBUG_SIGNAL */
#ifdef DEBUG_SIGNAL
#define DBG_MSG		stdout_debug
#else
#define DBG_MSG(x...)
#endif

int	__sigwait(const sigset_t *set, int *sig);
int	__sigwaitinfo(const sigset_t *set, siginfo_t *info);
int	__sigtimedwait(const sigset_t *set, siginfo_t *info,
		const struct timespec * timeout);

static void
sigcancel_handler(int sig __unused, siginfo_t *info __unused,
	ucontext_t *ucp __unused)
{
	struct pthread *curthread = tls_get_curthread();

	if (curthread->cancelflags & THR_CANCEL_AT_POINT)
		_pthread_testcancel();
	_thr_ast(curthread);
}

void
_thr_ast(struct pthread *curthread)
{
	if (!THR_IN_CRITICAL(curthread)) {
		if (__predict_false((curthread->flags &
		    (THR_FLAGS_NEED_SUSPEND | THR_FLAGS_SUSPENDED))
			== THR_FLAGS_NEED_SUSPEND))
			_thr_suspend_check(curthread);
	}
}

void
_thr_suspend_check(struct pthread *curthread)
{
	umtx_t cycle;

	/*
	 * Blocks SIGCANCEL which other threads must send.
	 */
	_thr_signal_block(curthread);

	/*
	 * Increase critical_count, here we don't use THR_LOCK/UNLOCK
	 * because we are leaf code, we don't want to recursively call
	 * ourself.
	 */
	curthread->critical_count++;
	THR_UMTX_LOCK(curthread, &(curthread)->lock);
	while ((curthread->flags & (THR_FLAGS_NEED_SUSPEND |
		THR_FLAGS_SUSPENDED)) == THR_FLAGS_NEED_SUSPEND) {
		curthread->cycle++;
		cycle = curthread->cycle;

		/* Wake the thread suspending us. */
		_thr_umtx_wake(&curthread->cycle, INT_MAX);

		/*
		 * if we are from pthread_exit, we don't want to
		 * suspend, just go and die.
		 */
		if (curthread->state == PS_DEAD)
			break;
		curthread->flags |= THR_FLAGS_SUSPENDED;
		THR_UMTX_UNLOCK(curthread, &(curthread)->lock);
		_thr_umtx_wait(&curthread->cycle, cycle, NULL, 0);
		THR_UMTX_LOCK(curthread, &(curthread)->lock);
		curthread->flags &= ~THR_FLAGS_SUSPENDED;
	}
	THR_UMTX_UNLOCK(curthread, &(curthread)->lock);
	curthread->critical_count--;

	_thr_signal_unblock(curthread);
}

void
_thr_signal_init(void)
{
	struct sigaction act;

	/* Install cancel handler. */
	SIGEMPTYSET(act.sa_mask);
	act.sa_flags = SA_SIGINFO | SA_RESTART;
	act.sa_sigaction = (__siginfohandler_t *)&sigcancel_handler;
	__sys_sigaction(SIGCANCEL, &act, NULL);
}

void
_thr_signal_deinit(void)
{
}

int
_sigaction(int sig, const struct sigaction * act, struct sigaction * oact)
{
	/* Check if the signal number is out of range: */
	if (sig < 1 || sig > _SIG_MAXSIG || sig == SIGCANCEL) {
		/* Return an invalid argument: */
		errno = EINVAL;
		return (-1);
	}

	return __sys_sigaction(sig, act, oact);
}

__strong_reference(_sigaction, sigaction);

int
_sigprocmask(int how, const sigset_t *set, sigset_t *oset)
{
	const sigset_t *p = set;
	sigset_t newset;

	if (how != SIG_UNBLOCK) {
		if (set != NULL) {
			newset = *set;
			SIGDELSET(newset, SIGCANCEL);
			p = &newset;
		}
	}
	return (__sys_sigprocmask(how, p, oset));
}

__strong_reference(_sigprocmask, sigprocmask);

int
_pthread_sigmask(int how, const sigset_t *set, sigset_t *oset)
{
	if (_sigprocmask(how, set, oset))
		return (errno);
	return (0);
}

__strong_reference(_pthread_sigmask, pthread_sigmask);

int
_sigsuspend(const sigset_t * set)
{
	struct pthread *curthread = tls_get_curthread();
	sigset_t newset;
	const sigset_t *pset;
	int oldcancel;
	int ret;

	if (SIGISMEMBER(*set, SIGCANCEL)) {
		newset = *set;
		SIGDELSET(newset, SIGCANCEL);
		pset = &newset;
	} else
		pset = set;

	oldcancel = _thr_cancel_enter(curthread);
	ret = __sys_sigsuspend(pset);
	_thr_cancel_leave(curthread, oldcancel);

	return (ret);
}

__strong_reference(_sigsuspend, sigsuspend);

int
__sigtimedwait(const sigset_t *set, siginfo_t *info,
	const struct timespec * timeout)
{
	struct pthread	*curthread = tls_get_curthread();
	sigset_t newset;
	const sigset_t *pset;
	int oldcancel;
	int ret;

	if (SIGISMEMBER(*set, SIGCANCEL)) {
		newset = *set;
		SIGDELSET(newset, SIGCANCEL);
		pset = &newset;
	} else
		pset = set;
	oldcancel = _thr_cancel_enter(curthread);
	ret = __sys_sigtimedwait(pset, info, timeout);
	_thr_cancel_leave(curthread, oldcancel);
	return (ret);
}

__strong_reference(__sigtimedwait, sigtimedwait);

int
__sigwaitinfo(const sigset_t *set, siginfo_t *info)
{
	struct pthread	*curthread = tls_get_curthread();
	sigset_t newset;
	const sigset_t *pset;
	int oldcancel;
	int ret;

	if (SIGISMEMBER(*set, SIGCANCEL)) {
		newset = *set;
		SIGDELSET(newset, SIGCANCEL);
		pset = &newset;
	} else
		pset = set;

	oldcancel = _thr_cancel_enter(curthread);
	ret = __sys_sigwaitinfo(pset, info);
	_thr_cancel_leave(curthread, oldcancel);
	return (ret);
}

__strong_reference(__sigwaitinfo, sigwaitinfo);

int
__sigwait(const sigset_t *set, int *sig)
{
	int s;

	s = __sigwaitinfo(set, NULL);
	if (s > 0) {
		*sig = s;
		return (0);
	}
	return (errno);
}

__strong_reference(__sigwait, sigwait);
