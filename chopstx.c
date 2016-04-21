/*
 * chopstx.c - Threads and only threads.
 *
 * Copyright (C) 2013, 2014, 2015, 2016
 *               Flying Stone Technology
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

#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

/*
 * Thread priority: higer has higher precedence.
 */
#if !defined(CHX_PRIO_MAIN_INIT)
#define CHX_PRIO_MAIN_INIT 1
#endif
#if !defined(CHX_FLAGS_MAIN)
#define CHX_FLAGS_MAIN 0
#endif

/* Constant for round robin scheduling.  */
#if !defined(PREEMPTION_USEC)
#define PREEMPTION_USEC 1000 /* 1ms */
#endif

#define MAX_PRIO (255+1)

/*
 * Exception priority: lower has higher precedence.
 *
 * Cortex-M3
 * =====================================
 * Prio 0x30: svc
 * ---------------------
 * Prio 0x40: thread temporarily inhibiting schedule for critical region
 * ...
 * Prio 0xb0: systick, external interrupt
 * Prio 0xc0: pendsv
 * =====================================
 *
 * Cortex-M0
 * =====================================
 * Prio 0x00: thread temporarily inhibiting schedule for critical region
 * ...
 * Prio 0x40: systick, external interrupt
 * Prio 0x80: pendsv
 * Prio 0x80: svc
 * =====================================
 */

#define CPU_EXCEPTION_PRIORITY_CLEAR         0

#if defined(__ARM_ARCH_6M__)
#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 0x00
/* ... */
#define CPU_EXCEPTION_PRIORITY_SYSTICK       CPU_EXCEPTION_PRIORITY_INTERRUPT
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     0x40
#define CPU_EXCEPTION_PRIORITY_PENDSV        0x80
#define CPU_EXCEPTION_PRIORITY_SVC           0x80 /* No use in this arch */
#elif defined(__ARM_ARCH_7M__)
#define CPU_EXCEPTION_PRIORITY_SVC           0x30

#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 0x40
/* ... */
#define CPU_EXCEPTION_PRIORITY_SYSTICK       CPU_EXCEPTION_PRIORITY_INTERRUPT
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     0xb0
#define CPU_EXCEPTION_PRIORITY_PENDSV        0xc0
#else
#error "no support for this arch"
#endif

/*
 * Lower layer architecture specific functions.
 *
 * system tick and interrupt
 */

/*
 * System tick
 */
/* SysTick registers.  */
static volatile uint32_t *const SYST_CSR = (uint32_t *const)0xE000E010;
static volatile uint32_t *const SYST_RVR = (uint32_t *const)0xE000E014;
static volatile uint32_t *const SYST_CVR = (uint32_t *const)0xE000E018;

static void
chx_systick_reset (void)
{
  *SYST_RVR = 0;
  *SYST_CVR = 0;
  *SYST_CSR = 7;
}

static void
chx_systick_reload (uint32_t ticks)
{
  *SYST_RVR = ticks;
  *SYST_CVR = 0;  /* write (any) to clear the counter to reload.  */
  *SYST_RVR = 0;
}

static uint32_t
chx_systick_get (void)
{
  return *SYST_CVR;
}

#ifndef MHZ
#define MHZ 72
#endif

static uint32_t usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}

/*
 * Interrupt Handling
 */

/* NVIC: Nested Vectored Interrupt Controller.  */
struct NVIC {
  volatile uint32_t ISER[8];
  uint32_t unused1[24];
  volatile uint32_t ICER[8];
  uint32_t unused2[24];
  volatile uint32_t ISPR[8];
  uint32_t unused3[24];
  volatile uint32_t ICPR[8];
  uint32_t unused4[24];
  volatile uint32_t IABR[8];
  uint32_t unused5[56];
  volatile uint32_t IPR[60];
};

static struct NVIC *const NVIC = (struct NVIC *const)0xE000E100;
#define NVIC_ISER(n)	(NVIC->ISER[n >> 5])
#define NVIC_ICER(n)	(NVIC->ICER[n >> 5])
#define NVIC_ICPR(n)	(NVIC->ICPR[n >> 5])
#define NVIC_IPR(n)	(NVIC->IPR[n >> 2])

#define USB_LP_CAN1_RX0_IRQn	 20


static void
chx_enable_intr (uint8_t irq_num)
{
  NVIC_ISER (irq_num) = 1 << (irq_num & 0x1f);
}

static void
chx_clr_intr (uint8_t irq_num)
{				/* Clear pending interrupt.  */
  NVIC_ICPR (irq_num) = 1 << (irq_num & 0x1f);
}

static void
chx_disable_intr (uint8_t irq_num)
{
  NVIC_ICER (irq_num) = 1 << (irq_num & 0x1f);
}

static void
chx_set_intr_prio (uint8_t n)
{
  unsigned int sh = (n & 3) << 3;

  NVIC_IPR (n) = (NVIC_IPR(n) & ~(0xFF << sh))
    | (CPU_EXCEPTION_PRIORITY_INTERRUPT << sh);
}

static volatile uint32_t *const ICSR = (uint32_t *const)0xE000ED04;

/* Priority control.  */
static uint32_t *const AIRCR = (uint32_t *const)0xE000ED0C;
static uint32_t *const SHPR2 = (uint32_t *const)0xE000ED1C;
static uint32_t *const SHPR3 = (uint32_t *const)0xE000ED20;

static void
chx_prio_init (void)
{
  *AIRCR = 0x05FA0000 | ( 5 << 8); /* PRIGROUP = 5, 2-bit:2-bit. */
  *SHPR2 = (CPU_EXCEPTION_PRIORITY_SVC << 24);
  *SHPR3 = ((CPU_EXCEPTION_PRIORITY_SYSTICK << 24)
	    | (CPU_EXCEPTION_PRIORITY_PENDSV << 16));
}

/**
 * chx_fatal - Fatal error point.
 * @err_code: Error code
 *
 * When it detects a coding error, this function will be called to
 * stop further execution of code.  It never returns.
 */
void
chx_fatal (uint32_t err_code)
{
  (void)err_code;
  for (;;);
}


/* RUNNING: the current thread. */
struct chx_thread *running;

struct chx_queue {
  struct chx_qh q;
  struct chx_spinlock lock;
};

/* READY: priority queue. */
static struct chx_queue q_ready;

/* Queue of threads waiting for timer.  */
static struct chx_queue q_timer;

/* Queue of threads which wait exit of some thread.  */
static struct chx_queue q_join;


/* Forward declaration(s). */
static void chx_request_preemption (uint16_t prio);
static void chx_timer_dequeue (struct chx_thread *tp);
static struct chx_thread *chx_timer_insert (struct chx_thread *tp,
					    uint32_t usec);



/**************/
static void chx_spin_init (struct chx_spinlock *lk)
{
  (void)lk;
}

static void chx_spin_lock (struct chx_spinlock *lk)
{
  (void)lk;
}

static void chx_spin_unlock (struct chx_spinlock *lk)
{
  (void)lk;
}

/* The thread context: specific to ARM Cortex-M3 now.  */
struct tcontext {
  uint32_t reg[9];	   /* r4, r5, r6, r7, r8, r9, r10, r11, r13(sp) */
};

/* Saved registers on the stack.  */
struct chx_stack_regs {
  uint32_t reg[8];	       /* r0, r1, r2, r3, r12, lr, pc, xpsr */
};

/*
 * Constants for ARM.
 */
#define REG_EXIT 4		/* R8 */
#define REG_SP 8

#define REG_R0   0
#define REG_LR   5
#define REG_PC   6
#define REG_XPSR 7

#define INITIAL_XPSR 0x01000000	/* T=1 */

/**************/
struct chx_pq {
  struct chx_pq *next, *prev;
  uint32_t                  : 4;
  uint32_t                  : 5;
  uint32_t                  : 6;
  uint32_t flag_is_proxy    : 1;
  uint32_t                  : 8;
  uint32_t prio             : 8;
  struct chx_qh *parent;
  uint32_t v;
};

struct chx_px {			/* inherits PQ */
  struct chx_pq *next, *prev;
  uint32_t                  : 4;
  uint32_t                  : 5;
  uint32_t                  : 6;
  uint32_t flag_is_proxy    : 1;
  uint32_t                  : 8;
  uint32_t prio             : 8;
  struct chx_qh *parent;
  uint32_t v;
  struct chx_thread *master;
  uint32_t *counter_p;
};

struct chx_thread {		/* inherits PQ */
  struct chx_pq *next, *prev;
  uint32_t state            : 4;
  uint32_t flag_detached    : 1;
  uint32_t flag_got_cancel  : 1;
  uint32_t flag_join_req    : 1;
  uint32_t flag_sched_rr    : 1;
  uint32_t flag_cancelable  : 1;
  uint32_t                  : 6;
  uint32_t flag_is_proxy    : 1;
  uint32_t prio_orig        : 8;
  uint32_t prio             : 8;
  struct chx_qh *parent;
  uint32_t v;
  struct tcontext tc;
  struct chx_mtx *mutex_list;
  struct chx_cleanup *clp;
};


static void
chx_cpu_sched_lock (void)
{
  if (running->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    {
#if defined(__ARM_ARCH_6M__)
      asm volatile ("cpsid	i" : : : "memory");
#else
      register uint32_t tmp = CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED;
      asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
#endif
    }
}

static void
chx_cpu_sched_unlock (void)
{
  if (running->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    {
#if defined(__ARM_ARCH_6M__)
      asm volatile ("cpsie	i" : : : "memory");
#else
      register uint32_t tmp = CPU_EXCEPTION_PRIORITY_CLEAR;
      asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
#endif
    }
}


/*
 * Double linked list handling.
 */

static int
ll_empty (struct chx_qh *q)
{
  return q == (struct chx_qh *)q->next;
}

static struct chx_pq *
ll_dequeue (struct chx_pq *pq)
{
  pq->next->prev = pq->prev;
  pq->prev->next = pq->next;
  pq->prev = pq->next = pq;
  return pq;
}

static void
ll_insert (struct chx_pq *pq0, struct chx_qh *q)
{
  struct chx_pq *pq = (struct chx_pq *)q;

  pq0->next = (struct chx_pq *)pq;
  pq0->prev = pq->prev;
  pq->prev->next = (struct chx_pq *)pq0;
  pq->prev = pq0;
}


static struct chx_pq *
ll_pop (struct chx_qh *q)
{
  if (q == (struct chx_qh *)q->next)
    return NULL;

  return ll_dequeue (q->next);
}

static void
ll_prio_push (struct chx_pq *pq0, struct chx_qh *q0)
{
  struct chx_pq *p;

  for (p = q0->next; p != (struct chx_pq *)q0; p = p->next)
    if (p->prio <= pq0->prio)
      break;

  pq0->parent = q0;
  ll_insert (pq0, (struct chx_qh *)p);
}

static void
ll_prio_enqueue (struct chx_pq *pq0, struct chx_qh *q0)
{
  struct chx_pq *p;

  for (p = q0->next; p != (struct chx_pq *)q0; p = p->next)
    if (p->prio < pq0->prio)
      break;

  pq0->parent = q0;
  ll_insert (pq0, (struct chx_qh *)p);
}


/*
 * Thread status.
 */
enum  {
  THREAD_RUNNING=0,
  THREAD_READY,
  THREAD_WAIT_MTX,
  THREAD_WAIT_CND,
  THREAD_WAIT_TIME,
  THREAD_WAIT_INT,
  THREAD_WAIT_POLL,
  THREAD_JOIN,
  /**/
  THREAD_EXITED=0x0E,
  THREAD_FINISHED=0x0F
};


static struct chx_thread *
chx_ready_pop (void)
{
  struct chx_thread *tp;

  chx_spin_lock (&q_ready.lock);
  tp = (struct chx_thread *)ll_pop (&q_ready.q);
  if (tp)
    tp->state = THREAD_RUNNING;
  chx_spin_unlock (&q_ready.lock);

  return tp;
}


static void
chx_ready_push (struct chx_thread *tp)
{
  chx_spin_lock (&q_ready.lock);
  tp->state = THREAD_READY;
  ll_prio_push ((struct chx_pq *)tp, &q_ready.q);
  chx_spin_unlock (&q_ready.lock);
}


static void
chx_ready_enqueue (struct chx_thread *tp)
{
  chx_spin_lock (&q_ready.lock);
  tp->state = THREAD_READY;
  ll_prio_enqueue ((struct chx_pq *)tp, &q_ready.q);
  chx_spin_unlock (&q_ready.lock);
}

static void __attribute__((naked, used))
idle (void)
{
#if defined(USE_WFI_FOR_IDLE)
  for (;;)
    asm volatile ("wfi" : : : "memory");
#else
  for (;;);
#endif
}


static void
chx_set_timer (struct chx_thread *tp, uint32_t ticks)
{
  if (tp == (struct chx_thread *)&q_timer.q)
    chx_systick_reload (ticks);
  else
    tp->v = ticks;
}

static struct chx_thread *
chx_timer_insert (struct chx_thread *tp, uint32_t usec)
{
  struct chx_pq *p;
  uint32_t ticks = usec_to_ticks (usec);
  uint32_t next_ticks = chx_systick_get ();

  for (p = q_timer.q.next; p != (struct chx_pq *)&q_timer.q; p = p->next)
    {
      if (ticks < next_ticks)
	{
	  ll_insert ((struct chx_pq *)tp, (struct chx_qh *)p);
	  chx_set_timer ((struct chx_thread *)tp->prev, ticks);
	  chx_set_timer (tp, (next_ticks - ticks));
	  break;
	}
      else
	{
	  ticks -= next_ticks;
	  next_ticks = p->v;
	}
    }

  if (p == (struct chx_pq *)&q_timer.q)
    {
      ll_insert ((struct chx_pq *)tp, (struct chx_qh *)p);
      chx_set_timer ((struct chx_thread *)tp->prev, ticks);
      chx_set_timer (tp, 1);	/* Non-zero for the last entry. */
    }

  return tp;
}


static void
chx_timer_dequeue (struct chx_thread *tp)
{
  struct chx_thread *tp_prev;

  chx_spin_lock (&q_timer.lock);
  tp_prev = (struct chx_thread *)tp->prev;
  if (tp_prev == (struct chx_thread *)&q_timer.q)
    {
      if (tp->next == (struct chx_pq *)&q_timer.q)
	chx_set_timer (tp_prev, 0); /* Cancel timer*/
      else
	{			/* Update timer.  */
	  uint32_t next_ticks = chx_systick_get () + tp->v;

	  chx_set_timer (tp_prev, next_ticks);
	}
    }
  else
    tp_prev->v += tp->v;
  ll_dequeue ((struct chx_pq *)tp);
  tp->v = 0;
  chx_spin_unlock (&q_timer.lock);
}


void
chx_timer_expired (void)
{
  struct chx_thread *tp;
  uint16_t prio = 0;			/* Use uint16_t here. */

  chx_spin_lock (&q_timer.lock);
  if ((tp = (struct chx_thread *)ll_pop (&q_timer.q)))
    {
      uint32_t next_tick = tp->v;

      chx_ready_enqueue (tp);
      if (tp == running)	/* tp->flag_sched_rr == 1 */
	prio = MAX_PRIO;
      else
	if ((uint16_t)tp->prio > prio)
	  prio = (uint16_t)tp->prio;

      if (!ll_empty (&q_timer.q))
	{
	  struct chx_thread *tp_next;

	  for (tp = (struct chx_thread *)q_timer.q.next;
	       tp != (struct chx_thread *)&q_timer.q && next_tick == 0;
	       tp = tp_next)
	    {
	      next_tick = tp->v;
	      tp_next = (struct chx_thread *)tp->next;
	      ll_dequeue ((struct chx_pq *)tp);
	      chx_ready_enqueue (tp);
	      if (tp == running)
		prio = MAX_PRIO;
	      else
		if ((uint16_t)tp->prio > prio)
		  prio = (uint16_t)tp->prio;
	    }

	  if (!ll_empty (&q_timer.q))
	    chx_set_timer ((struct chx_thread *)&q_timer.q, next_tick);
	}
    }

  chx_request_preemption (prio);
  chx_spin_unlock (&q_timer.lock);
}

static chopstx_intr_t *intr_top;
static struct chx_spinlock intr_lock;

void
chx_handle_intr (void)
{
  chopstx_intr_t *intr;
  register uint32_t irq_num;

  asm volatile ("mrs	%0, IPSR\n\t"
		"sub	%0, #16"   /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : /* no input */ : "memory");
  chx_disable_intr (irq_num);
  chx_spin_lock (&intr_lock);
  for (intr = intr_top; intr; intr = intr->next)
    if (intr->irq_num == irq_num)
      break;

  if (intr)
    {
      intr->ready++;
      if (intr->tp != running)
	{
	  if (intr->tp->state == THREAD_WAIT_POLL)
	    {
	      chx_timer_dequeue (intr->tp);
	      chx_ready_enqueue (intr->tp);
	      chx_request_preemption (intr->tp->prio);
	    }
	  else if (intr->tp->state == THREAD_WAIT_INT)
	    {
	      chx_ready_enqueue (intr->tp);
	      chx_request_preemption (intr->tp->prio);
	    }
	}
    }
  chx_spin_unlock (&intr_lock);
}

void
chx_systick_init (void)
{
  chx_systick_reset ();

  if ((CHX_FLAGS_MAIN & CHOPSTX_SCHED_RR))
    {
      chx_cpu_sched_lock ();
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (running, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
      chx_cpu_sched_unlock ();
    }
}

chopstx_t chopstx_main;

void
chx_init (struct chx_thread *tp)
{
  chx_spin_init (&intr_lock);
  chx_prio_init ();
  memset (&tp->tc, 0, sizeof (tp->tc));
  q_ready.q.next = q_ready.q.prev = (struct chx_pq *)&q_ready.q;
  chx_spin_init (&q_ready.lock);
  q_timer.q.next = q_timer.q.prev = (struct chx_pq *)&q_timer.q;
  chx_spin_init (&q_timer.lock);
  q_join.q.next = q_join.q.prev = (struct chx_pq *)&q_join.q;
  chx_spin_init (&q_join.lock);
  tp->next = tp->prev = (struct chx_pq *)tp;
  tp->mutex_list = NULL;
  tp->clp = NULL;
  tp->state = THREAD_RUNNING;
  tp->flag_got_cancel = tp->flag_join_req = 0;
  tp->flag_cancelable = 1;
  tp->flag_sched_rr = (CHX_FLAGS_MAIN & CHOPSTX_SCHED_RR)? 1 : 0;
  tp->flag_detached = (CHX_FLAGS_MAIN & CHOPSTX_DETACHED)? 1 : 0;
  tp->flag_is_proxy = 0;
  tp->prio_orig = CHX_PRIO_MAIN_INIT;
  tp->prio = 0;
  tp->parent = NULL;
  tp->v = 0;
  running = tp;

  if (CHX_PRIO_MAIN_INIT >= CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    chx_cpu_sched_lock ();

  tp->prio = CHX_PRIO_MAIN_INIT;

  chopstx_main = (chopstx_t)tp;
}


/**
 * chopstx_main_init - initialize main thread
 * @prio: priority
 *
 * Initialize main thread with @prio.
 * The thread main is created with priority CHX_PRIO_MAIN_INIT,
 * and it runs with that priority until this routine will is called.
 */
void
chopstx_main_init (chopstx_prio_t prio)
{
  struct chx_thread *tp = (struct chx_thread *)chopstx_main;

  tp->prio_orig = prio;

  if (prio >= CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    chx_cpu_sched_lock ();

  tp->prio = prio;
}


static void
chx_request_preemption (uint16_t prio)
{
  if (running == NULL || (uint16_t)running->prio < prio)
    {
      *ICSR = (1 << 28);
      asm volatile ("" : : : "memory");
    }
}


#define CHX_SLEEP 0
#define CHX_YIELD 1

/*
 * chx_sched: switch to another thread.
 *
 * There are two cases:
 *   YIELD=0 (SLEEP): Current RUNNING thread is already connected to
 *                    something (mutex, cond, intr, etc.)
 *   YIELD=1 (YIELD): Current RUNNING thread is active,
 *                    it is needed to be enqueued to READY queue.
 *
 * For Cortex-M0, this should be AAPCS-compliant function entry, so we
 * put "noinline" attribute.
 *
 * 	AAPCS: ARM Architecture Procedure Call Standard
 *
 */
static int __attribute__ ((naked, noinline))
chx_sched (uint32_t yield)
{
  register struct chx_thread *tp asm ("r0");

#if defined(__ARM_ARCH_7M__)
  asm volatile (
	"svc	#0\n\t"
	"bx	lr"
	: "=r" (tp) : "0" (yield): "memory");
#else
  register uint32_t arg_yield asm ("r1");

  /* Build stack data as if it were an exception entry.  */
  /*
   * r0:  0                     scratch
   * r1:  0                     scratch
   * r2:  0                     scratch
   * r3:  0                     scratch
   * r12: 0                     scratch
   * lr   as-is
   * pc:  return address (= lr)
   * psr: INITIAL_XPSR          scratch
   */
  asm ("mov	r1, lr\n\t"
       "mov	r2, r1\n\t"
       "mov	r3, #128\n\t"
       "lsl	r3, #17\n\t"
       "push	{r1, r2, r3}\n\t"
       "mov	r1, #0\n\t"
       "mov	r2, r1\n\t"
       "mov	r3, r1\n\t"
       "push	{r1, r2, r3}\n\t"
       "push	{r1, r2}"
       : /* no output*/
       : "r" (yield)
       : "r1", "r2", "r3", "memory");

  /* Save registers onto CHX_THREAD struct.  */
  asm ("mov	r1, r0\n\t"
       "ldr	r2, =running\n\t"
       "ldr	r0, [r2]\n\t"
       "add	r0, #20\n\t"
       "stm	r0!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mov	r6, sp\n\t"
       "stm	r0!, {r2, r3, r4, r5, r6}\n\t"
       "sub	r0, #56"
       : "=r" (tp), "=r" (arg_yield)
       : "0" (yield)
       : "r2", "r3", "r4", "r5", "r6", "r7", "memory");

  if (arg_yield)
    {
      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      chx_ready_enqueue (tp);
    }

  tp = chx_ready_pop ();
  if (tp && tp->flag_sched_rr)
    {
      chx_spin_lock (&q_timer.lock);
      tp = chx_timer_insert (tp, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
    }

  asm volatile (/* Now, r0 points to the thread to be switched.  */
		/* Put it to *running.  */
		"ldr	r1, =running\n\t"
		/* Update running.  */
		"str	r0, [r1]\n\t"
		"cmp	r0, #0\n\t"
		"bne	0f\n\t"

		/* Spawn an IDLE thread.  */
		"ldr	r1, =__main_stack_end__\n\t"
		"mov	sp, r1\n\t"
		"ldr	r0, =idle\n\t"	     /* PC = idle */
		/**/
		/* Unmask interrupts.  */
		"cpsie	i\n\t"
		"bx	r0\n"

		/* Normal context switch */
	"0:\n\t"
		/**/
		"add	r0, #20\n\t"
		"ldm	r0!, {r4, r5, r6, r7}\n\t"
		"ldm	r0!, {r1, r2, r3}\n\t"
		"mov	r8, r1\n\t"
		"mov	r9, r2\n\t"
		"mov	r10, r3\n\t"
		"ldm	r0!, {r1, r2}\n\t"
		"mov	r11, r1\n\t"
		"mov	sp, r2\n\t"
		"sub	r0, #45\n\t"
		"ldrb	r1, [r0]\n\t" /* ->PRIO field.  */
		"cmp	r1, #247\n\t"
		"bhi	1f\n\t"	/* Leave interrupt disabled if >= 248 */
		/**/
		/* Unmask interrupts.  */
		"cpsie	i\n"
		/**/
	"1:\n\t"
		/*
		  0:  r0
		  4:  r1
		  8:  r2
		  12: r3
		  16: r12
		  20: lr
		  24: pc
		  28: psr
		  32: possibly exists for alignment
		  [28 or 32] <-- pc           
		*/
		"ldr	r0, [sp, #28]\n\t"
		"lsl	r1, r0, #23\n\t"
		"bcc	2f\n\t"
		/**/
		"msr	APSR_nzcvq, r0\n\t"
		"ldr	r0, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r0, r1\n\t"	/* Ensure Thumb-mode */
		"str	r0, [sp, #32]\n\t"
		/**/
		"ldr	r0, [sp, #20]\n\t"
		"mov	lr, r0\n\t"
		"ldr	r0, [sp, #16]\n\t"
		"mov	r12, r0\n\t"
		"pop	{r0, r1, r2, r3}\n\t"
		"add	sp, #16\n\t"
		"pop	{pc}\n"
	"2:\n\t"
		"msr	APSR_nzcvq, r0\n\t"
		"ldr	r0, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r0, r1\n\t"	/* Ensure Thumb-mode */
		"str	r0, [sp, #28]\n\t"
		/**/
		"ldr	r0, [sp, #20]\n\t"
		"mov	lr, r0\n\t"
		"ldr	r0, [sp, #16]\n\t"
		"mov	r12, r0\n\t"
		"pop	{r0, r1, r2, r3}\n\t"
		"add	sp, #12\n\t"
		"pop	{pc}"
		: "=r" (tp)		/* Return value in R0 */
		: "0" (tp)
		: "memory");
#endif

  return (uint32_t)tp;
}


/* The RETVAL is saved into register R8.  */
static void __attribute__((noreturn))
chx_exit (void *retval)
{
  register uint32_t r8 asm ("r8");
  struct chx_pq *p;

  asm volatile ("mov	%0, %1" : "=r" (r8) : "r" (retval));

  chx_cpu_sched_lock ();
  if (running->flag_join_req)
    {		       /* wake up a thread which requests to join */
      chx_spin_lock (&q_join.lock);
      for (p = q_join.q.next; p != (struct chx_pq *)&q_join.q; p = p->next)
	if (p->v == (uint32_t)running)
	  {			/* should be one at most. */
	    struct chx_thread *tp = (struct chx_thread *)p;

	    ll_dequeue (p);
	    if (tp->flag_is_proxy)
	      {
		struct chx_px *px = (struct chx_px *)p;

		(*px->counter_p)++;
		tp = px->master;
		if (tp->state == THREAD_WAIT_POLL)
		  {
		    chx_timer_dequeue (tp);
		    goto wakeup;
		  }
	      }
	    else
	      {
		((struct chx_stack_regs *)tp->tc.reg[REG_SP])->reg[REG_R0] = 0;
	      wakeup:
		chx_ready_enqueue (tp);
	      }
	    break;
	  }
      chx_spin_unlock (&q_join.lock);
    }

  if (running->flag_sched_rr)
    chx_timer_dequeue (running);
  if (running->flag_detached)
    running->state = THREAD_FINISHED;
  else
    running->state = THREAD_EXITED;
  asm volatile ("" : : "r" (r8) : "memory");
  chx_sched (CHX_SLEEP);
  /* never comes here. */
  for (;;);
}


static chopstx_prio_t
chx_mutex_unlock (chopstx_mutex_t *mutex)
{
  struct chx_thread *tp;
  chopstx_prio_t prio = 0;

  mutex->owner = NULL;
  running->mutex_list = mutex->list;
  mutex->list = NULL;

  tp = (struct chx_thread *)ll_pop (&mutex->q);
  if (tp)
    {
      uint16_t newprio = running->prio_orig;
      chopstx_mutex_t *m;

      chx_ready_enqueue (tp);

      /* Examine mutexes we hold, and determine new priority for running.  */
      for (m = running->mutex_list; m; m = m->list)
	if (!ll_empty (&m->q)
	    && ((struct chx_thread *)(m->q.next))->prio > newprio)
	  newprio = ((struct chx_thread *)m->q.next)->prio;
      /* Then, assign it.  */
      running->prio = newprio;

      if (prio < tp->prio)
	prio = tp->prio;
    }

  return prio;
}

#define CHOPSTX_PRIO_MASK ((1 << CHOPSTX_PRIO_BITS) - 1)

typedef void *(voidfunc) (void *);

extern void cause_link_time_error_unexpected_size_of_struct_chx_thread (void);

/**
 * chopstx_create - Create a thread
 * @flags_and_prio: Flags and priority
 * @stack_addr: Stack address
 * @stack_size: Size of stack
 * @thread_entry: Entry function of new thread
 * @arg: Argument to the thread entry function
 *
 * Create a thread.  Returns thread ID.
 */
chopstx_t
chopstx_create (uint32_t flags_and_prio,
		uint32_t stack_addr, size_t stack_size,
		voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  void *stack;
  struct chx_stack_regs *p;
  chopstx_prio_t prio = (flags_and_prio & CHOPSTX_PRIO_MASK);

  if (CHOPSTX_THREAD_SIZE != sizeof(struct chx_thread))
    cause_link_time_error_unexpected_size_of_struct_chx_thread ();

  if (stack_size < sizeof (struct chx_thread) + 8 * sizeof (uint32_t))
    chx_fatal (CHOPSTX_ERR_THREAD_CREATE);

  stack = (void *)(stack_addr + stack_size - sizeof (struct chx_thread)
		   - sizeof (struct chx_stack_regs));
  memset (stack, 0, sizeof (struct chx_stack_regs));
  tp = (struct chx_thread *)(stack + sizeof (struct chx_stack_regs));
  p = (struct chx_stack_regs *)stack;
  p->reg[REG_R0] = (uint32_t)arg;
  p->reg[REG_LR] = (uint32_t)chopstx_exit;
  p->reg[REG_PC] = (uint32_t)thread_entry;
  p->reg[REG_XPSR] = INITIAL_XPSR;

  memset (&tp->tc, 0, sizeof (tp->tc));
  tp->tc.reg[REG_SP] = (uint32_t)stack;
  tp->next = tp->prev = (struct chx_pq *)tp;
  tp->mutex_list = NULL;
  tp->clp = NULL;
  tp->state = THREAD_EXITED;
  tp->flag_got_cancel = tp->flag_join_req = 0;
  tp->flag_cancelable = 1;
  tp->flag_sched_rr = (flags_and_prio & CHOPSTX_SCHED_RR)? 1 : 0;
  tp->flag_detached = (flags_and_prio & CHOPSTX_DETACHED)? 1 : 0;
  tp->flag_is_proxy = 0;
  tp->prio_orig = tp->prio = prio;
  tp->parent = NULL;
  tp->v = 0;

  chx_cpu_sched_lock ();
  chx_ready_enqueue (tp);
  if (tp->prio > running->prio)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();

  return (chopstx_t)tp;
}

/*
 * Internal timer uses SYSTICK and it has rather smaller upper limit.
 * Besides, we should check cancel condition of the thread
 * periodically.  Thus, we don't let the thread sleep too long, but
 * let it loops.
 *
 * 200ms is the upper limit.
 *
 * The caller should make a loop with chx_snooze.
 */
#define MAX_USEC_FOR_TIMER (200*1000)
static int
chx_snooze (uint32_t state, uint32_t *usec_p)
{
  uint32_t usec = *usec_p;
  uint32_t usec0;
  int r;

  if (usec == 0)
    {
      chx_cpu_sched_unlock ();
      return -1;
    }

  usec0 = (usec > MAX_USEC_FOR_TIMER) ? MAX_USEC_FOR_TIMER: usec;
  if (running->flag_sched_rr)
    chx_timer_dequeue (running);

  chx_spin_lock (&q_timer.lock);
  running->state = state;
  chx_timer_insert (running, usec0);
  chx_spin_unlock (&q_timer.lock);
  r = chx_sched (CHX_SLEEP);
  if (r >= 0)
    *usec_p -= usec0;

  return r;
}

/**
 * chopstx_usec_wait_var - Sleep for micro seconds (specified by variable)
 * @var: Pointer to usec
 *
 * Sleep for micro seconds, specified by @var.
 * Another thread can clear @var to stop the caller going into sleep.
 */
void
chopstx_usec_wait_var (uint32_t *var)
{
  int r = 0;

  do
    {
      chopstx_testcancel ();
      chx_cpu_sched_lock ();
      r = chx_snooze (THREAD_WAIT_TIME, var);
    }
  while (r == 0);
}


/**
 * chopstx_usec_wait - Sleep for micro seconds
 * @usec: number of micro seconds
 *
 * Sleep for @usec.
 */
void
chopstx_usec_wait (uint32_t usec)
{
  chopstx_usec_wait_var (&usec);
}


/**
 * chopstx_mutex_init - Initialize the mutex
 * @mutex: Mutex
 *
 * Initialize @mutex.
 */
void
chopstx_mutex_init (chopstx_mutex_t *mutex)
{
  chx_spin_init (&mutex->lock);
  mutex->q.next = mutex->q.prev = (struct chx_pq *)&mutex->q;
  mutex->list = NULL;
}


/**
 * chopstx_mutex_lock - Lock the mutex
 * @mutex: Mutex
 *
 * Lock @mutex.
 */
void
chopstx_mutex_lock (chopstx_mutex_t *mutex)
{
  struct chx_thread *tp = running;

  while (1)
    {
      chopstx_mutex_t *m = mutex;
      struct chx_thread *owner;

      chx_cpu_sched_lock ();
      chx_spin_lock (&m->lock);
      if (m->owner == NULL)
	{
	  /* The mutex is acquired.  */
	  m->owner = tp;
	  m->list = tp->mutex_list;
	  tp->mutex_list = m;
	  chx_spin_unlock (&m->lock);
	  chx_cpu_sched_unlock ();
	  break;
	}

      owner = m->owner;
      while (1)
	{			/* Priority inheritance.  */
	  owner->prio = tp->prio;
	  if (owner->state == THREAD_READY || owner->state == THREAD_WAIT_CND)
	    {
	      ll_prio_enqueue (ll_dequeue ((struct chx_pq *)owner),
			       owner->parent);
	      break;
	    }
	  else if (owner->state == THREAD_WAIT_MTX)
	    {
	      ll_prio_enqueue (ll_dequeue ((struct chx_pq *)owner),
			       owner->parent);
	      owner = m->owner;
	      continue;
	    }
	  else if (owner->state == THREAD_JOIN
		   || owner->state == THREAD_WAIT_TIME
		   || owner->state == THREAD_WAIT_POLL)
	    {
	      ((struct chx_stack_regs *)owner->tc.reg[REG_SP])->reg[REG_R0] = -1;
	      chx_ready_enqueue (owner);
	      break;
	    }
	  else
	    break;
	  /* Assume it's UP no case of: ->state==RUNNING on SMP??? */
	}

      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      ll_prio_enqueue ((struct chx_pq *)tp, &mutex->q);
      tp->state = THREAD_WAIT_MTX;
      chx_spin_unlock (&mutex->lock);
      chx_sched (CHX_SLEEP);
    }
}


/**
 * chopstx_mutex_unlock - Unlock the mutex
 * @mutex: Mutex
 *
 * Unlock @mutex.
 */
void
chopstx_mutex_unlock (chopstx_mutex_t *mutex)
{
  chopstx_prio_t prio;

  chx_cpu_sched_lock ();
  chx_spin_lock (&mutex->lock);
  prio = chx_mutex_unlock (mutex);
  chx_spin_unlock (&mutex->lock);
  if (prio > running->prio)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();
}


/**
 * chopstx_cond_init - Initialize the condition variable
 * @cond: Condition variable
 *
 * Initialize @cond.
 */
void
chopstx_cond_init (chopstx_cond_t *cond)
{
  chx_spin_init (&cond->lock);
  cond->q.next = cond->q.prev = (struct chx_pq *)&cond->q;
}


/**
 * chopstx_cond_wait - Wait on the condition variable
 * @cond: Condition variable
 * @mutex: Associated mutex
 *
 * Wait for @cond with @mutex.
 */
void
chopstx_cond_wait (chopstx_cond_t *cond, chopstx_mutex_t *mutex)
{
  struct chx_thread *tp = running;

  chopstx_testcancel ();
  chx_cpu_sched_lock ();

  if (mutex)
    {
      chx_spin_lock (&mutex->lock);
      chx_mutex_unlock (mutex);
      chx_spin_unlock (&mutex->lock);
    }

  if (tp->flag_sched_rr)
    chx_timer_dequeue (tp);
  chx_spin_lock (&cond->lock);
  ll_prio_enqueue ((struct chx_pq *)tp, &cond->q);
  tp->state = THREAD_WAIT_CND;
  chx_spin_unlock (&cond->lock);
  chx_sched (CHX_SLEEP);

  if (mutex)
    chopstx_mutex_lock (mutex);
}

static int
chx_wakeup_from_cond_wait (struct chx_thread *tp)
{
  int yield = 0;

  if (tp->flag_is_proxy)
    {
      struct chx_px *px = (struct chx_px *)tp;

      (*px->counter_p)++;
      tp = px->master;
      if (tp->state == THREAD_WAIT_POLL)
	{
	  chx_timer_dequeue (tp);
	  ((struct chx_stack_regs *)tp->tc.reg[REG_SP])->reg[REG_R0] = -1;
	  goto wakeup;
	}
    }
  else
    {
    wakeup:
      chx_ready_enqueue (tp);
      if (tp->prio > running->prio)
	yield = 1;
    }

  return yield;
}

/**
 * chopstx_cond_signal - Wake up a thread waiting on the condition variable
 * @cond: Condition variable
 *
 * Wake up a thread waiting on @cond.
 */
void
chopstx_cond_signal (chopstx_cond_t *cond)
{
  struct chx_thread *tp;
  int yield = 0;

  chx_cpu_sched_lock ();
  chx_spin_lock (&cond->lock);
  tp = (struct chx_thread *)ll_pop (&cond->q);
  if (tp)
    yield = chx_wakeup_from_cond_wait (tp);
  chx_spin_unlock (&cond->lock);
  if (yield)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();
}


/**
 * chopstx_cond_broadcast - Wake up all waiting on the condition variable
 * @cond: Condition Variable
 *
 * Wake up all threads waiting on @cond.
 */
void
chopstx_cond_broadcast (chopstx_cond_t *cond)
{
  struct chx_thread *tp;
  int yield = 0;

  chx_cpu_sched_lock ();
  chx_spin_lock (&cond->lock);
  while ((tp = (struct chx_thread *)ll_pop (&cond->q)))
    yield |= chx_wakeup_from_cond_wait (tp);
  chx_spin_unlock (&cond->lock);
  if (yield)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();
}


/**
 * chopstx_cond_hook - Register a proxy to wait on the confition variable
 * @px: Proxy to a thread
 * @cond: Condition Variable
 * @mutex: Associated mutex
 * @func: Function to evaluate the condition
 * @arg: Argument to the @func
 *
 * If @func with @arg returns 0, register @px to wait for @cond with @mutex.
 */
void
chopstx_cond_hook (chopstx_px_t *px, chopstx_cond_t *cond,
		   chopstx_mutex_t *mutex, int (*func) (void *), void *arg)
{
  chopstx_testcancel ();

  if (mutex)
    chopstx_mutex_lock (mutex);

  if ((*func) (arg) != 0)
    (*px->counter_p)++;
  else
    { /* Condition doesn't met.
       * Register the proxy to wait for the condition.
       */
      chx_cpu_sched_lock ();
      chx_spin_lock (&cond->lock);
      ll_prio_enqueue ((struct chx_pq *)px, &cond->q);
      chx_spin_unlock (&cond->lock);
      chx_cpu_sched_unlock ();
    }

  if (mutex)
    chopstx_mutex_unlock (mutex);
}

/**
 * chopstx_cond_unhook - de-register a proxy to wait on the confition variable
 * @px: Proxy to a thread
 * @cond: Condition Variable

 * If @px is on @cond, dequeue it from it.
 */
void
chopstx_cond_unhook (chopstx_px_t *px, chopstx_cond_t *cond)
{
  chx_cpu_sched_lock ();
  if (px->parent == &cond->q)
    {
      ll_dequeue ((struct chx_pq *)px);
      px->parent = NULL;
    }
  chx_cpu_sched_unlock ();
}


/**
 * chopstx_claim_irq - Claim interrupt request to handle by this thread
 * @intr: Pointer to INTR structure
 * @irq_num: IRQ Number (hardware specific)
 *
 * Claim interrupt @intr with @irq_num for this thread.
 */
void
chopstx_claim_irq (chopstx_intr_t *intr, uint8_t irq_num)
{
  intr->irq_num = irq_num;
  intr->tp = running;
  intr->ready = 0;
  chx_cpu_sched_lock ();
  chx_disable_intr (irq_num);
  chx_set_intr_prio (irq_num);
  chx_spin_lock (&intr_lock);
  intr->next = intr_top;
  intr_top = intr;
  chx_spin_unlock (&intr_lock);
  chx_cpu_sched_unlock ();
}


/**
 * chopstx_realease_irq - Unregister interrupt request
 * @intr0: Interrupt request to be unregistered
 *
 * Release the interrupt request specified by @intr0.
 */
void
chopstx_release_irq (chopstx_intr_t *intr0)
{
  chopstx_intr_t *intr, *intr_prev;

  chx_cpu_sched_lock ();
  chx_enable_intr (intr0->irq_num);
  chx_spin_lock (&intr_lock);
  intr_prev = intr_top;
  for (intr = intr_top; intr; intr = intr->next)
    if (intr == intr0)
      break;

  if (intr == intr_top)
    intr_top = intr_top->next;
  else
    intr_prev->next = intr->next;
  chx_spin_unlock (&intr_lock);
  chx_cpu_sched_unlock ();
}


static void
chx_release_irq_thread (struct chx_thread *tp)
{
  chopstx_intr_t *intr, *intr_prev;

  chx_cpu_sched_lock ();
  chx_spin_lock (&intr_lock);
  intr_prev = intr_top;
  for (intr = intr_top; intr; intr = intr->next)
    {
      if (intr->tp == tp)
	break;
      intr_prev = intr;
    }

  if (intr)
    {
      chx_enable_intr (intr->irq_num);
      if (intr == intr_top)
	intr_top = intr_top->next;
      else
	intr_prev->next = intr->next;
    }
  chx_spin_unlock (&intr_lock);
  chx_cpu_sched_unlock ();
}


/**
 * chopstx_intr_wait - Wait for interrupt request from hardware
 * @intr: Pointer to INTR structure
 *
 * Wait for the interrupt @intr to be occured.
 */
void
chopstx_intr_wait (chopstx_intr_t *intr)
{
  chopstx_testcancel ();
  chx_cpu_sched_lock ();
  if (intr->ready == 0)
    {
      chx_enable_intr (intr->irq_num);
      if (running->flag_sched_rr)
	chx_timer_dequeue (running);
      running->state = THREAD_WAIT_INT;
      running->parent = NULL;
      running->v = 0;
      chx_sched (CHX_SLEEP);
      chx_clr_intr (intr->irq_num);
    }
  else
    chx_cpu_sched_unlock ();
  intr->ready--;
}


/**
 * chopstx_cleanup_push - Register a clean-up
 * @clp: Pointer to clean-up structure
 *
 * Register a clean-up structure.
 */
void
chopstx_cleanup_push (struct chx_cleanup *clp)
{
  clp->next = running->clp;
  running->clp = clp;
}

/**
 * chopstx_cleanup_pop - Release a clean-up
 * @execute: Execute the clen-up function on release
 *
 * Unregister a clean-up structure.  When @execute is non-zero, the
 * clean-up will be executed.
 */
void
chopstx_cleanup_pop (int execute)
{
  struct chx_cleanup *clp = running->clp;

  if (clp)
    {
      running->clp = clp->next;
      if (execute)
	clp->routine (clp->arg);
    }
}


/**
 * chopstx_exit - Terminate the execution of thread
 * @retval: Return value (to be caught by a joining thread)
 *
 * Calling this function terminates the execution of thread, after
 * calling clean up functions.  If the calling thread still holds
 * mutexes, they will be released.  If the calling thread claiming
 * IRQ, it will be released, too.  This function never returns.
 */
void
chopstx_exit (void *retval)
{
  struct chx_mtx *m, *m_next;
  struct chx_cleanup *clp = running->clp;

  running->clp = NULL;
  while (clp)
    {
      clp->routine (clp->arg);
      clp = clp->next;
    }

  /* Release all mutexes this thread still holds.  */
  for (m = running->mutex_list; m; m = m_next)
    {
      m_next = m->list;

      chx_cpu_sched_lock ();
      chx_spin_lock (&m->lock);
      chx_mutex_unlock (m);
      chx_spin_unlock (&m->lock);
      chx_cpu_sched_unlock ();
    }

  chx_release_irq_thread (running);
  chx_exit (retval);
}


/**
 * chopstx_join - join with a terminated thread
 * @thd: Thread to wait
 * @ret: Pointer to void * to store return value
 *
 * Waits for the thread of @thd to terminate.
 */
void
chopstx_join (chopstx_t thd, void **ret)
{
  struct chx_thread *tp = (struct chx_thread *)thd;

  /*
   * We don't offer deadlock detection.  It's users' responsibility.
   */

  chx_cpu_sched_lock ();
  if (tp->flag_detached)
    {
      chx_cpu_sched_unlock ();
      chx_fatal (CHOPSTX_ERR_JOIN);
    }

  if (tp->state != THREAD_EXITED)
    {
      if (running->flag_sched_rr)
	chx_timer_dequeue (running);
      chx_spin_lock (&q_join.lock);
      ll_prio_enqueue ((struct chx_pq *)running, &q_join.q);
      running->v = (uint32_t)tp;
      running->state = THREAD_JOIN;
      chx_spin_unlock (&q_join.lock);
      tp->flag_join_req = 1;
      if (tp->prio < running->prio)
	{
	  tp->prio = running->prio;
	  if (tp->state == THREAD_READY
	      || tp->state == THREAD_WAIT_MTX || tp->state == THREAD_WAIT_CND)
	    ll_prio_enqueue (ll_dequeue ((struct chx_pq *)tp), tp->parent);
	}
      chx_sched (CHX_SLEEP);
    }
  else
    chx_cpu_sched_unlock ();

  tp->state = THREAD_FINISHED;
  if (ret)
    *ret = (void *)tp->tc.reg[REG_EXIT]; /* R8 */
}


/**
 * chopstx_wakeup_usec_wait - wakeup the sleeping thread for timer
 * @thd: Thread to be awakened
 *
 * Canceling the timer, wake up the sleeping thread.
 * No return value.
 */
void
chopstx_wakeup_usec_wait (chopstx_t thd)
{
  struct chx_thread *tp = (struct chx_thread *)thd;
  int yield = 0;

  chx_cpu_sched_lock ();
  if (tp->state == THREAD_WAIT_TIME)
    {
      ((struct chx_stack_regs *)tp->tc.reg[REG_SP])->reg[REG_R0] = -1;
      chx_timer_dequeue (tp);
      chx_ready_enqueue (tp);
      if (tp->prio > running->prio)
	yield = 1;
    }
  if (yield)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();
}

/**
 * chopstx_cancel - request a cancellation to a thread
 * @thd: Thread to be canceled
 *
 * This function requests a cancellation of a thread @thd.
 * No return value.
 */
void
chopstx_cancel (chopstx_t thd)
{
  struct chx_thread *tp = (struct chx_thread *)thd;
  struct chx_stack_regs *p;
  int yield = 0;

  /* Assume it's UP, it's *never*: tp->state==RUNNING on SMP??? */
  chx_cpu_sched_lock ();
  tp->flag_got_cancel = 1;
  if (!tp->flag_cancelable)
    {
      chx_cpu_sched_unlock ();
      return;
    }

  /* Cancellation points: cond_wait, intr_wait, and usec_wait.  */
  if (tp->state == THREAD_WAIT_CND || tp->state == THREAD_WAIT_INT
      || tp->state == THREAD_WAIT_TIME || tp->state == THREAD_WAIT_POLL)
    {
      /* Throw away registers on stack and direct to chopstx_exit.  */
      /* This is pretty much violent, but it works.  */
      p = (struct chx_stack_regs *)tp->tc.reg[REG_SP];
      p->reg[REG_R0] = CHOPSTX_EXIT_CANCELED;
      p->reg[REG_PC] = (uint32_t)chopstx_exit;

      if (tp->state == THREAD_WAIT_CND)
	ll_dequeue ((struct chx_pq *)tp);
      else if (tp->state == THREAD_WAIT_TIME || tp->state == THREAD_WAIT_POLL)
	chx_timer_dequeue (tp);

      chx_ready_enqueue (tp);
      if (tp->prio > running->prio)
	yield = 1;
    }
  if (yield)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();
}


/**
 * chopstx_testcancel - catch pending cancellation request
 *
 * Calling chopstx_testcancel creates a cancellation point.
 * No return value.  If the thread is canceled, this function
 * does not return.
 */
void
chopstx_testcancel (void)
{
  if (running->flag_cancelable && running->flag_got_cancel)
    chopstx_exit ((void *)CHOPSTX_EXIT_CANCELED_IN_SYNC);
}


/**
 * chopstx_setcancelstate - set cancelability state
 * @cancel_disable: 0 to enable cancelation, otherwise disabled.
 * 
 * Calling chopstx_setcancelstate sets cancelability state.
 *
 * Returns old state which is 0 when it was enabled.
 */
int
chopstx_setcancelstate (int cancel_disable)
{
  int old_state = !running->flag_cancelable;

  running->flag_cancelable = (cancel_disable == 0);
  chopstx_testcancel ();
  return old_state;
}

static void
chx_proxy_init (struct chx_px *px, uint32_t *cp)
{
  px->next = px->prev = (struct chx_pq *)px;
  px->flag_is_proxy = 1;
  px->prio = running->prio;
  px->parent = NULL;
  px->v = 0;
  px->master = running;
  px->counter_p = cp;
}


int
chopstx_poll (uint32_t *usec_p, int n, ...)
{
  uint32_t counter = 0;
  int i;
  va_list ap;
  struct chx_px px[n];
  chopstx_poll_fnc pollfnc;

  chopstx_testcancel ();

  for (i = 0; i < n; i++)
    chx_proxy_init (&px[i], &counter);

  va_start (ap, n);
  for (i = 0; i < n; i++)
    {
      pollfnc = va_arg (ap, chopstx_poll_fnc);
      (*pollfnc) (0, &px[i]);
    }
  va_end (ap);

  chx_cpu_sched_lock ();
  if (counter)
    chx_cpu_sched_unlock ();
  else
    {
      int r;

      chx_cpu_sched_unlock ();
      do
	{
	  chopstx_testcancel ();
	  chx_cpu_sched_lock ();
	  r = chx_snooze (THREAD_WAIT_POLL, usec_p);
	}
      while (r == 0);
    }

  va_start (ap, n);
  for (i = 0; i < n; i++)
    {
      pollfnc = va_arg (ap, chopstx_poll_fnc);
      (*pollfnc) (1, &px[i]);
    }
  va_end (ap);

  return counter;		/* Bitmap??? */
}

/*
 * Lower layer architecture specific exception handling entries.
 * 
 */

void __attribute__ ((naked))
preempt (void)
{
  register struct chx_thread *tp asm ("r0");
  register struct chx_thread *cur asm ("r1");

  asm volatile (
#if defined(__ARM_ARCH_6M__)
	"cpsid	i\n\t"
#else
	"msr	BASEPRI, r0\n\t"
#endif
	"ldr	r2, =running\n\t"
	"ldr	r0, [r2]\n\t"
	"mov	r1, r0"
	: "=r" (tp), "=r" (cur)
	: /* no input */
	: "r2");

  if (!cur)
    /* It's idle thread.  It's ok to clobber registers.  */
    ;
  else
    {
      /* Save registers onto CHX_THREAD struct.  */
      asm volatile (
	"add	%0, #20\n\t"
	"stm	%0!, {r4, r5, r6, r7}\n\t"
	"mov	r2, r8\n\t"
	"mov	r3, r9\n\t"
	"mov	r4, r10\n\t"
	"mov	r5, r11\n\t"
	"mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
	"stm	%0!, {r2, r3, r4, r5, r6}"
	: "=r" (cur)
	: "0" (cur)
          /*
	   * Memory clobber constraint here is not accurate, but this
	   * works.  R7 keeps its value, but having "r7" here prevents
	   * use of R7 before this asm statement.
	   */ 
	: "r2", "r3", "r4", "r5", "r6", "r7", "memory");

      if (tp)
	{
	  if (tp->flag_sched_rr)
	    {
	      if (tp->state == THREAD_RUNNING)
		{
		  chx_timer_dequeue (tp);
		  chx_ready_enqueue (tp);
		}
	      /*
	       * It may be THREAD_READY after chx_timer_expired.
	       * Then, do nothing.
	       */
	    }
	  else
	    chx_ready_push (tp);
	  running = NULL;
	}
    }

  /* Registers on stack (PSP): r0, r1, r2, r3, r12, lr, pc, xpsr */

  tp = chx_ready_pop ();
  if (tp && tp->flag_sched_rr)
    {
      chx_spin_lock (&q_timer.lock);
      tp = chx_timer_insert (tp, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
    }

  asm volatile (
    ".L_CONTEXT_SWITCH:\n\t"
	/* Now, r0 points to the thread to be switched.  */
	/* Put it to *running.  */
	"ldr	r1, =running\n\t"
	/* Update running.  */
	"str	r0, [r1]\n\t"
#if defined(__ARM_ARCH_6M__)
	"cmp	r0, #0\n\t"
	"beq	1f\n\t"
#else
	"cbz	r0, 1f\n\t"
#endif
	/**/
	"add	r0, #20\n\t"
	"ldm	r0!, {r4, r5, r6, r7}\n\t"
#if defined(__ARM_ARCH_6M__)
	"ldm	r0!, {r1, r2, r3}\n\t"
	"mov	r8, r1\n\t"
	"mov	r9, r2\n\t"
	"mov	r10, r3\n\t"
	"ldm	r0!, {r1, r2}\n\t"
	"mov	r11, r1\n\t"
	"msr	PSP, r2\n\t"
#else
	"ldr	r8, [r0], #4\n\t"
	"ldr	r9, [r0], #4\n\t"
	"ldr	r10, [r0], #4\n\t"
	"ldr	r11, [r0], #4\n\t"
	"ldr	r1, [r0], #4\n\t"
	"msr	PSP, r1\n\t"
#endif
	"sub	r0, #45\n\t"
	"ldrb	r1, [r0]\n\t" /* ->PRIO field.  */
	"mov	r0, #0\n\t"
	"cmp	r1, #247\n\t"
	"bhi	0f\n\t"	/* Leave interrupt disabled if >= 248 */
	/**/
	/* Unmask interrupts.  */
#if defined(__ARM_ARCH_6M__)
	"cpsie	i\n"
#else
	"msr	BASEPRI, r0\n"
#endif
	/**/
    "0:\n\t"
	"sub	r0, #3\n\t" /* EXC_RETURN to a thread with PSP */
	"bx	r0\n"
    "1:\n\t"
	/* Spawn an IDLE thread.  */
	"ldr	r0, =__main_stack_end__-32\n\t"
	"msr	PSP, r0\n\t"
	"mov	r1, #0\n\t"
	"mov	r2, #0\n\t"
	"mov	r3, #0\n\t"
	"stm	r0!, {r1, r2, r3}\n\t"
	"stm	r0!, {r1, r2, r3}\n\t"
	"ldr	r1, =idle\n\t"	     /* PC = idle */
	"mov	r2, #0x010\n\t"
	"lsl	r2, r2, #20\n\t" /* xPSR = T-flag set (Thumb) */
	"stm	r0!, {r1, r2}\n\t"
	/**/
	/* Unmask interrupts.  */
	"mov	r0, #0\n\t"
#if defined(__ARM_ARCH_6M__)
	"cpsie	i\n\t"
#else
	"msr	BASEPRI, r0\n"
#endif
	/**/
	"sub	r0, #3\n\t" /* EXC_RETURN to a thread with PSP */
	"bx	r0"
	: /* no output */ : "r" (tp) : "memory");
}

#if defined(__ARM_ARCH_7M__)
/*
 * System call: switch to another thread.
 * There are two cases:
 *   ORIG_R0=0 (SLEEP): Current RUNNING thread is already connected to
 *                      something (mutex, cond, intr, etc.)
 *   ORIG_R0=1 (YIELD): Current RUNNING thread is active,
 *                      it is needed to be enqueued to READY queue.
 */
void __attribute__ ((naked))
svc (void)
{
  register struct chx_thread *tp asm ("r0");
  register uint32_t orig_r0 asm ("r1");

  asm ("ldr	r1, =running\n\t"
       "ldr	r0, [r1]\n\t"
       "add	r1, r0, #20\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r1!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r1!, {r2, r3, r4, r5, r6}\n\t"
       "ldr	r1, [r6]"
       : "=r" (tp), "=r" (orig_r0)
       : /* no input */
       : "r2", "r3", "r4", "r5", "r6", "memory");

  if (orig_r0)			/* yield */
    {
      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      chx_ready_enqueue (tp);
      running = NULL;
    }

  tp = chx_ready_pop ();
  if (tp && tp->flag_sched_rr)
    {
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (tp, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
    }

  asm volatile (
	"b	.L_CONTEXT_SWITCH"
	: /* no output */ : "r" (tp) : "memory");
}
#endif
