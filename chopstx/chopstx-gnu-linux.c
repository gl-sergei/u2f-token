/*
 * chopstx-gnu-linux.c - Threads and only threads: Arch specific code
 *                       for GNU/Linux emulation
 *
 * Copyright (C) 2017 Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * receipents of GNU GPL by a written offer.
 *
 */

#include <unistd.h>
#include <ucontext.h>
#include <signal.h>
#include <sys/time.h>

static sigset_t ss_cur;

static void
chx_systick_reset (void)
{
  const struct itimerval it = { {0, 0}, {0, 0} };

  setitimer (ITIMER_REAL, &it, 0);
}

static void
chx_systick_reload (uint32_t ticks)
{
  struct itimerval it;

  it.it_value.tv_sec = 0;
  it.it_value.tv_usec = (ticks / MHZ);
  it.it_interval.tv_sec = 0;
  it.it_interval.tv_usec = 0;

  setitimer (ITIMER_REAL, &it, 0);
}

static uint32_t
chx_systick_get (void)
{
  struct itimerval it;
  getitimer (ITIMER_REAL, &it);
  return it.it_value.tv_usec * 72;
}

static uint32_t
usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}


static void
chx_enable_intr (uint8_t irq_num)
{
  sigdelset (&ss_cur, irq_num);
}

static void
chx_clr_intr (uint8_t irq_num)
{				/* Clear pending interrupt.  */
  (void)irq_num;
}

static void
chx_disable_intr (uint8_t irq_num)
{
  sigaddset (&ss_cur, irq_num);
}

static void
chx_set_intr_prio (uint8_t n)
{
  (void)n;
}

static void
chx_prio_init (void)
{
}

static void
chx_cpu_sched_lock (void)
{
  sigset_t ss;

  sigfillset (&ss);
  pthread_sigmask (SIG_BLOCK, &ss, &ss_cur);
}

static void
chx_cpu_sched_unlock (void)
{
  pthread_sigmask (SIG_SETMASK, &ss_cur, NULL);
}

static void
idle (void)
{
  for (;;)
    pause ();
}

void
chx_handle_intr (uint32_t irq_num)
{
  struct chx_pq *p;

  chx_disable_intr (irq_num);
  chx_spin_lock (&q_intr.lock);
  for (p = q_intr.q.next; p != (struct chx_pq *)&q_intr.q; p = p->next)
    if (p->v == irq_num)
      {			/* should be one at most. */
	struct chx_px *px = (struct chx_px *)p;

	ll_dequeue (p);
	chx_wakeup (p);
	chx_spin_unlock (&q_intr.lock);
	chx_request_preemption (px->master->prio);
	return;
      }
  chx_spin_unlock (&q_intr.lock);
}


static ucontext_t idle_tc;
static char idle_stack[4096];

struct chx_thread main_thread;

void
chx_sigmask (ucontext_t *uc)
{
  /* Modify oldmask to SS_CUR, so that the signal mask will
   * be set to SS_CUR.
   *
   * In user-level, sigset_t is big, but only the first word
   * is used by the kernel.
   */
  memcpy (&uc->uc_sigmask, &ss_cur, sizeof (uint64_t));
}

static void
sigalrm_handler (int sig, siginfo_t *siginfo, void *arg)
{
  extern void chx_timer_expired (void);
  ucontext_t *uc = arg;
  (void)sig;
  (void)siginfo;
  chx_timer_expired ();
  chx_sigmask (uc);
}

static void
chx_init_arch (struct chx_thread *tp)
{
  struct sigaction sa;

  sigemptyset (&ss_cur);

  sa.sa_sigaction = sigalrm_handler;
  sigfillset (&sa.sa_mask);
  sa.sa_flags = SA_SIGINFO|SA_RESTART;
  sigaction (SIGALRM, &sa, NULL); 

  getcontext (&idle_tc);
  idle_tc.uc_stack.ss_sp = idle_stack;
  idle_tc.uc_stack.ss_size = sizeof (idle_stack);
  idle_tc.uc_link = NULL;
  makecontext (&idle_tc, idle, 0);

  getcontext (&tp->tc);
}

static void
chx_request_preemption (uint16_t prio)
{
  struct chx_thread *tp, *tp_prev;
  ucontext_t *tcp;

  if (running && (uint16_t)running->prio >= prio)
    return;

  /* Change the context to another thread with higher priority.  */
  tp = tp_prev = running;
  if (tp)
    {
      if (tp->flag_sched_rr)
	{
	  if (tp->state == THREAD_RUNNING)
	    {
	      chx_timer_dequeue (tp);
	      chx_ready_enqueue (tp);
	    }
	}
      else
	chx_ready_push (tp);
      running = NULL;
    }

  tp = running = chx_ready_pop ();
  if (tp)
    {
      tcp = &tp->tc;
      if (tp->flag_sched_rr)
	{
	  chx_spin_lock (&q_timer.lock);
	  tp = chx_timer_insert (tp, PREEMPTION_USEC);
	  chx_spin_unlock (&q_timer.lock);
	}
    }
  else
    tcp = &idle_tc;

  if (tp_prev)
    {
      /*
       * The swapcontext implementation may reset sigmask in the
       * middle of its execution, unfortunately.  It is best if
       * sigmask restore is done at the end of the routine, but we
       * can't assume that.
       *
       * Thus, there might be a race condition with regards to the
       * user context TCP, if signal mask is cleared and signal comes
       * in.  To avoid this situation, we block signals.
       *
       * We don't need to fill the mask here.  It keeps the condition
       * of blocking signals before&after swapcontext call.  It is
       * done by the signal mask for sigaction, the initial creation
       * of the thread, and the condition of chx_sched function which
       * mandates holding cpu_sched_lock.
       */
      swapcontext (&tp_prev->tc, tcp);
    }
  else if (tp)
    {
      setcontext (tcp);
    }
}

/*
 * chx_sched: switch to another thread.
 *
 * There are two cases:
 *   YIELD=0 (SLEEP): Current RUNNING thread is already connected to
 *                    something (mutex, cond, intr, etc.)
 *   YIELD=1 (YIELD): Current RUNNING thread is active,
 *                    it is needed to be enqueued to READY queue.
 *
 * Returns:
 *          1 on wakeup by others.
 *          0 on normal wakeup.
 *         -1 on cancellation.
 */
static uintptr_t
chx_sched (uint32_t yield)
{
  struct chx_thread *tp, *tp_prev;
  uintptr_t v;
  ucontext_t *tcp;

  tp = tp_prev = running;
  if (yield)
    {
      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      chx_ready_enqueue (tp);
    }

  running = tp = chx_ready_pop ();
  if (tp)
    {
      v = tp->v;
      if (tp->flag_sched_rr)
	{
	  chx_spin_lock (&q_timer.lock);
	  tp = chx_timer_insert (tp, PREEMPTION_USEC);
	  chx_spin_unlock (&q_timer.lock);
	}
      tcp = &tp->tc;
    }
  else
    {
      v = 0;
      tcp = &idle_tc;
    }

  swapcontext (&tp_prev->tc, tcp);
  chx_cpu_sched_unlock ();
  return v;
}

static void __attribute__((__noreturn__))
chx_thread_start (voidfunc thread_entry, void *arg)
{
  chx_cpu_sched_unlock ();
  thread_entry (arg);
  chopstx_exit (0);
}

static struct chx_thread *
chopstx_create_arch (uintptr_t stack_addr, size_t stack_size,
		     voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  tp = malloc (sizeof (struct chx_thread));
  if (!tp)
    chx_fatal (CHOPSTX_ERR_THREAD_CREATE);

  /*
   * Calling getcontext with sched_lock held, the context is with
   * signal blocked.  The sigmask will be cleared in chx_thread_start.
   */
  chx_cpu_sched_lock ();
  getcontext (&tp->tc);
  tp->tc.uc_stack.ss_sp = (void *)stack_addr;
  tp->tc.uc_stack.ss_size = stack_size;
  tp->tc.uc_link = NULL;

  makecontext (&tp->tc, (void (*)(void))chx_thread_start,
	       4, thread_entry, arg);
  chx_cpu_sched_unlock ();
  return tp;
}
