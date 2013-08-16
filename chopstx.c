/*
 * chopstx.c - Threads and only threads.
 *
 * Copyright (C) 2013 Flying Stone Technology
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

/*
 * Thread priority: higer has higher precedence.
 */
#if !defined(CHX_PRIO_MAIN)
#define CHX_PRIO_MAIN 1
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
 * Prio 0x30: svc
 * ---------------------
 * Prio 0x40: thread temporarily inhibiting schedule for critical region
 * Prio 0x50: systick
 * Prio 0x60: external interrupt
 * Prio 0x70: pendsv
 */

#define CPU_EXCEPTION_PRIORITY_CLEAR         0

#define CPU_EXCEPTION_PRIORITY_SVC           0x30

#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 0x40
#define CPU_EXCEPTION_PRIORITY_SYSTICK       0x50
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     0x60
#define CPU_EXCEPTION_PRIORITY_PENDSV        0x70

/**
 * chx_fatal - Fatal error point.
 * @err_code: Error code
 *
 * At runtime, detected an coding error which should be known at least
 * at compile time (or on design phase), this function will be called
 * to stop further execution of code.  It never returns.
 */
void
chx_fatal (uint32_t err_code)
{
  (void)err_code;
  for (;;);
}


/* RUNNING: the current thread. */
struct chx_thread *running;

/* Double linked list operations.  */
struct chx_dll {
  struct chx_thread *next, *prev;
};


struct chx_queue {
  struct chx_thread *next, *prev;
  struct chx_spinlock lock;
};

/* READY: priority queue. */
static struct chx_queue q_ready;

/* Queue of threads waiting for timer.  */
static struct chx_queue q_timer;

/* Queue of threads which have been exited. */
static struct chx_queue q_exit;

/* Queue of threads which wait exit of some thread.  */
static struct chx_queue q_join;


/* Forward declaration(s). */
static void chx_request_preemption (void);
static void chx_timer_dequeue (struct chx_thread *tp);
static void chx_timer_insert (struct chx_thread *tp, uint32_t usec);



/**************/
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
  uint32_t reg[9];	   /* r4, r5, r6, r7, r8, r9, r10, r11, r13 */
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

/*
 * NVIC: Nested Vectored Interrupt Controller
 */
struct NVIC {
  uint32_t ISER[8];
  uint32_t unused1[24];
  uint32_t ICER[8];
  uint32_t unused2[24];
  uint32_t ISPR[8];
  uint32_t unused3[24];
  uint32_t ICPR[8];
  uint32_t unused4[24];
  uint32_t IABR[8];
  uint32_t unused5[56];
  uint32_t IPR[60];
};

static struct NVIC *const NVICBase = (struct NVIC *const)0xE000E100;
#define NVIC_ISER(n)	(NVICBase->ISER[n >> 5])
#define NVIC_ICER(n)	(NVICBase->ICER[n >> 5])
#define NVIC_ICPR(n)	(NVICBase->ICPR[n >> 5])
#define NVIC_IPR(n)	(NVICBase->IPR[n >> 2])

#define USB_LP_CAN1_RX0_IRQn	 20

/*
 * SysTick registers.
 */
static volatile uint32_t *const SYST_CSR = (uint32_t *const)0xE000E010;
static volatile uint32_t *const SYST_RVR = (uint32_t *const)0xE000E014;
static volatile uint32_t *const SYST_CVR = (uint32_t *const)0xE000E018;

#define MHZ 72

static uint32_t usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}
/**************/

struct chx_thread {
  struct chx_thread *next, *prev;
  struct tcontext tc;
  uint32_t state            : 4;
  uint32_t flag_detached    : 1;
  uint32_t flag_got_cancel  : 1;
  uint32_t flag_join_req    : 1;
  uint32_t flag_sched_rr    : 1;
  uint32_t                  : 8;
  uint32_t prio_orig        : 8;
  uint32_t prio             : 8;
  uint32_t v;
  struct chx_mtx *mutex_list;
  struct chx_cleanup *clp;
};


static void
chx_cpu_sched_lock (void)
{
  if (running->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    {
      register uint32_t tmp = CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED;
      asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
    }
}

static void
chx_cpu_sched_unlock (void)
{
  if (running->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    {
      register uint32_t tmp = CPU_EXCEPTION_PRIORITY_CLEAR;
      asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
    }
}


/*
 * Double linked list handling.
 */

static int
ll_empty (void *head)
{
  struct chx_thread *l = (struct chx_thread *)head;

  return (struct chx_thread *)l == l->next;
}

static struct chx_thread *
ll_dequeue (struct chx_thread *tp)
{
  tp->next->prev = tp->prev;
  tp->prev->next = tp->next;
  tp->prev = tp->next = tp;
  return tp;
}

static void
ll_insert (struct chx_thread *tp0, void *head)
{
  struct chx_thread *tp = (struct chx_thread *)head;

  tp0->next = tp;
  tp0->prev = tp->prev;
  tp->prev->next = tp0;
  tp->prev = tp0;
}


static struct chx_thread *
ll_pop (void *head)
{
  struct chx_thread *l = (struct chx_thread *)head;
  struct chx_thread *tp0 = l->next;

  if (tp0 == l)
    return NULL;

  return ll_dequeue (tp0);
}

static void
ll_prio_push (struct chx_thread *tp0, void *head)
{
  struct chx_thread *l = (struct chx_thread *)head; 
  struct chx_thread *tp;

  for (tp = l->next; tp != l; tp = tp->next)
    if (tp->prio <= tp0->prio)
      break;

  ll_insert (tp0, tp);
}

static void
ll_prio_enqueue (struct chx_thread *tp0, void *head)
{
  struct chx_thread *l = (struct chx_thread *)head;
  struct chx_thread *tp;

  for (tp = l->next; tp != l; tp = tp->next)
    if (tp->prio < tp0->prio)
      break;

  ll_insert (tp0, tp);
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
  tp = ll_pop (&q_ready);
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
  ll_prio_push (tp, &q_ready);
  chx_spin_unlock (&q_ready.lock);
}


static void
chx_ready_enqueue (struct chx_thread *tp)
{
  chx_spin_lock (&q_ready.lock);
  tp->state = THREAD_READY;
  ll_prio_enqueue (tp, &q_ready);
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


/* Registers on stack (PSP): r0, r1, r2, r3, r12, lr, pc, xpsr */
static void __attribute__ ((naked, used))
sched (void)
{
  register struct chx_thread *tp asm ("r0");

  tp = chx_ready_pop ();
  if (tp && tp->flag_sched_rr)
    {
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (tp, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
    }
  asm volatile (/* Now, r0 points to the thread to be switched.  */
		/* Put it to *running.  */
		"ldr	r1, =running\n\t"
		/* Update running.  */
		"str	r0, [r1]\n\t"
		"cbz	r0, 1f\n\t"
		/**/
		"add	r0, #8\n\t"
		"ldm	r0!, {r4, r5, r6, r7}\n\t"
		"ldr	r8, [r0], #4\n\t"
		"ldr	r9, [r0], #4\n\t"
		"ldr	r10, [r0], #4\n\t"
		"ldr	r11, [r0], #4\n\t"
		"ldr	r1, [r0], #4\n\t"
		"msr	PSP, r1\n\t"
		"ldrb	r1, [r0, #3]\n\t" /* ->PRIO field.  */
		"cmp	r1, #247\n\t"
		"bhi	0f\n\t"	/* Leave interrupt disabled if >= 248 */
		/**/
		"mov	r0, #0\n\t"
		"msr	BASEPRI, r0\n"	      /* Unmask interrupts.  */
		/**/
	"0:\n\t"
		"sub	r0, #3\n\t" /* EXC_RETURN to a thread with PSP */
		"bx	r0\n"
	"1:\n\t"
		/* Spawn an IDLE thread.  */
		"ldr	r0, =__main_stack_end__\n\t"
		"msr	MSP, r0\n\t"
		"mov	r0, #0\n\t"
		"mov	r1, #0\n\t"
		"ldr	r2, =idle\n\t"	     /* PC = idle */
		"mov	r3, #0x01000000\n\t" /* xPSR = T-flag set (Thumb) */
		"push	{r0, r1, r2, r3}\n\t"
		"mov	r0, #0\n\t"
		"mov	r1, #0\n\t"
		"mov	r2, #0\n\t"
		"mov	r3, #0\n\t"
		"push	{r0, r1, r2, r3}\n"
		/**/
		"mov	r0, #0\n\t"
		"msr	BASEPRI, r0\n\t"	      /* Unmask interrupts.  */
		/**/
		"sub	r0, #7\n\t" /* EXC_RETURN to a thread with MSP */
		"bx	r0\n"
		: /* no output */ : "r" (tp) : "memory");
}

void __attribute__ ((naked))
preempt (void)
{
  register struct chx_thread *tp asm ("r0");

  asm ("ldr	r1, =running\n\t"
       "ldr	r0, [r1]\n\t"
       "cbnz	r0, 0f\n\t"
       /* It's idle which was preempted.  Discard saved registers on stack.  */
       "ldr	r1, =__main_stack_end__\n\t"
       "msr	MSP, r1\n\t"
       "b	sched\n"
  "0:\n\t"
       "add	r1, r0, #8\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r1!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r1, {r2, r3, r4, r5, r6}"
       : "=r" (tp)
       : /* no input */
       : "r1", "r2", "r3", "r4", "r5", "r6", "cc", "memory");

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

  asm volatile ("b	sched"
		: /* no output */: /* no input */ : "memory");
}


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
       "add	r1, r0, #8\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r1!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r1, {r2, r3, r4, r5, r6}\n\t"
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

  asm volatile ("b	sched"
		: /* no output */: /* no input */ : "memory");
}


static void
chx_set_timer (struct chx_thread *q, uint32_t ticks)
{
  if (q == (struct chx_thread *)&q_timer)
    {
      *SYST_RVR = ticks;
      *SYST_CVR = 0;  /* write (any) to clear the counter to reload.  */
      *SYST_RVR = 0;
    }
  else
    q->v = ticks;
}

static void
chx_timer_insert (struct chx_thread *tp, uint32_t usec)
{
  uint32_t ticks = usec_to_ticks (usec);
  uint32_t next_ticks = *SYST_CVR;
  struct chx_thread *q;

  for (q = q_timer.next; q != (struct chx_thread *)&q_timer; q = q->next)
    {
      if (ticks < next_ticks)
	{
	  ll_insert (tp, q);
	  chx_set_timer (tp->prev, ticks);
	  chx_set_timer (tp, (next_ticks - ticks));
	  break;
	}
      else
	{
	  ticks -= next_ticks;
	  next_ticks = q->v;
	}
    }

  if (q == (struct chx_thread *)&q_timer)
    {
      ll_insert (tp, q);
      chx_set_timer (tp->prev, ticks);
      chx_set_timer (tp, 1);	/* Non-zero for the last entry. */
    }
}


static void
chx_timer_dequeue (struct chx_thread *tp)
{
  struct chx_thread *tp_prev;

  chx_spin_lock (&q_timer.lock);
  tp_prev = tp->prev;
  if (tp_prev == (struct chx_thread *)&q_timer)
    {
      if (tp->next == (struct chx_thread *)&q_timer)
	chx_set_timer (tp_prev, 0); /* Cancel timer*/
      else
	{			/* Update timer.  */
	  uint32_t next_ticks = *SYST_CVR + tp->v;

	  chx_set_timer (tp_prev, next_ticks);
	}
    }
  else
    tp_prev->v += tp->v;
  ll_dequeue (tp);
  tp->v = 0;
  chx_spin_unlock (&q_timer.lock);
}


void
chx_timer_expired (void)
{
  struct chx_thread *tp;
  uint16_t prio = 0;			/* Use uint16_t here. */

  chx_spin_lock (&q_timer.lock);
  if ((tp = ll_pop (&q_timer)))
    {
      uint32_t next_tick = tp->v;

      chx_ready_enqueue (tp);
      if (tp == running)	/* tp->flag_sched_rr == 1 */
	prio = MAX_PRIO;
      else
	if ((uint16_t)tp->prio > prio)
	  prio = (uint16_t)tp->prio;

      if (!ll_empty (&q_timer))
	{
	  struct chx_thread *tp_next;

	  for (tp = q_timer.next;
	       tp != (struct chx_thread *)&q_timer && next_tick == 0;
	       tp = tp_next)
	    {
	      next_tick = tp->v;
	      tp_next = tp->next;
	      ll_dequeue (tp);
	      chx_ready_enqueue (tp);
	      if (tp == running)
		prio = MAX_PRIO;
	      else
		if ((uint16_t)tp->prio > prio)
		  prio = (uint16_t)tp->prio;
	    }

	  if (!ll_empty (&q_timer))
	    chx_set_timer ((struct chx_thread *)&q_timer, next_tick);
	}
    }

  if (running == NULL || (uint16_t)running->prio < prio)
    chx_request_preemption ();
  chx_spin_unlock (&q_timer.lock);
}


static void
chx_enable_intr (uint8_t irq_num)
{
  NVIC_ISER (irq_num) = 1 << (irq_num & 0x1f);
}

static void
chx_disable_intr (uint8_t irq_num)
{
  NVIC_ICER (irq_num) = 1 << (irq_num & 0x1f);
  /* Clear pending, too.  */
  NVIC_ICPR (irq_num) = 1 << (irq_num & 0x1f);
}


static void
chx_set_intr_prio (uint8_t n)
{
  unsigned int sh = (n & 3) << 3;

  NVIC_IPR (n) = (NVIC_IPR(n) & ~(0xFF << sh))
    | (CPU_EXCEPTION_PRIORITY_INTERRUPT << sh);
}

static chopstx_intr_t *intr_top;
static volatile uint32_t *const ICSR = (uint32_t *const)0xE000ED04;

void
chx_handle_intr (void)
{
  chopstx_intr_t *intr;
  register uint32_t irq_num;

  asm volatile ("mrs	%0, IPSR\n\t"
		"sub	%0, #16"   /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : /* no input */ : "memory");
  chx_disable_intr (irq_num);
  for (intr = intr_top; intr; intr = intr->next)
    if (intr->irq_num == irq_num)
      break;

  if (intr)
    {
      intr->ready++;
      if (intr->tp != running && intr->tp->state == THREAD_WAIT_INT)
	{
	  chx_ready_enqueue (intr->tp);
	  if (running == NULL || running->prio < intr->tp->prio)
	    chx_request_preemption ();
	}
    }
}

void
chx_systick_init (void)
{
  *SYST_RVR = 0;
  *SYST_CVR = 0;
  *SYST_CSR = 7;

  if ((CHX_FLAGS_MAIN & CHOPSTX_SCHED_RR))
    {
      chx_cpu_sched_lock ();
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (running, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
      chx_cpu_sched_unlock ();
    }
}

static uint32_t *const AIRCR = (uint32_t *const)0xE000ED0C;
static uint32_t *const SHPR2 = (uint32_t *const)0xE000ED1C;
static uint32_t *const SHPR3 = (uint32_t *const)0xE000ED20;

chopstx_t chopstx_main;

void
chx_init (struct chx_thread *tp)
{
  *AIRCR = 0x05FA0000 | ( 5 << 8); /* PRIGROUP = 5, 2-bit:2-bit. */
  *SHPR2 = (CPU_EXCEPTION_PRIORITY_SVC << 24);
  *SHPR3 = ((CPU_EXCEPTION_PRIORITY_SYSTICK << 24)
	    | (CPU_EXCEPTION_PRIORITY_PENDSV << 16));

  memset (&tp->tc, 0, sizeof (tp->tc));
  q_ready.next = q_ready.prev = (struct chx_thread *)&q_ready;
  q_timer.next = q_timer.prev = (struct chx_thread *)&q_timer;
  q_exit.next = q_exit.prev = (struct chx_thread *)&q_exit;
  q_join.next = q_join.prev = (struct chx_thread *)&q_join;
  tp->next = tp->prev = tp;
  tp->mutex_list = NULL;
  tp->clp = NULL;
  tp->state = THREAD_RUNNING;
  tp->flag_got_cancel = tp->flag_join_req = 0;
  tp->flag_sched_rr = (CHX_FLAGS_MAIN & CHOPSTX_SCHED_RR)? 1 : 0;
  tp->flag_detached = (CHX_FLAGS_MAIN & CHOPSTX_DETACHED)? 1 : 0;
  tp->prio_orig = CHX_PRIO_MAIN;
  tp->prio = 0;
  tp->v = 0;
  running = tp;

  if (CHX_PRIO_MAIN >= CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    chx_cpu_sched_lock ();

  tp->prio = CHX_PRIO_MAIN;

  chopstx_main = (chopstx_t)tp;
}


static void
chx_request_preemption (void)
{
  *ICSR = (1 << 28);
  asm volatile ("" : : : "memory");
}

#define CHX_SLEEP 0
#define CHX_YIELD 1

static void
chx_sched (uint32_t arg)
{
  register uint32_t r0 asm ("r0") = arg;

  asm volatile ("svc	#0" : : "r" (r0): "memory");
}


/* The RETVAL is saved into register R8.  */
static void __attribute__((noreturn))
chx_exit (void *retval)
{
  register uint32_t r8 asm ("r8") = (uint32_t)retval;
  struct chx_thread *q;

  chx_cpu_sched_lock ();
  if (running->flag_join_req)
    {		       /* wake up a thread which requests to join */
      chx_spin_lock (&q_join.lock);
      for (q = q_join.next; q != (struct chx_thread *)&q_join; q = q->next)
	if (q->v == (uint32_t)running)
	  {			/* should be one at most. */
	    ll_dequeue (q);
	    chx_ready_enqueue (q);
	    break;
	  }
      chx_spin_unlock (&q_join.lock);
    }

  if (running->flag_sched_rr)
    chx_timer_dequeue (running);
  chx_spin_lock (&q_exit.lock);
  ll_insert (running, &q_exit);
  if (running->flag_detached)
    running->state = THREAD_FINISHED;
  else
    running->state = THREAD_EXITED;
  chx_spin_unlock (&q_exit.lock);
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

  tp = ll_pop (&mutex->q);
  if (tp)
    {
      uint16_t newprio = running->prio_orig;
      chopstx_mutex_t *m;

      chx_ready_enqueue (tp);

      /* Examine mutexes we hold, and determine new priority for running.  */
      for (m = running->mutex_list; m; m = m->list)
	if (!ll_empty (&m->q) && m->q.next->prio > newprio)
	  newprio = m->q.next->prio;
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
 * Create a thread.
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
  p = (struct chx_stack_regs *)stack;
  p->reg[REG_R0] = (uint32_t)arg;
  p->reg[REG_LR] = (uint32_t)chopstx_exit;
  p->reg[REG_PC] = (uint32_t)thread_entry;
  p->reg[REG_XPSR] = INITIAL_XPSR;

  tp = (struct chx_thread *)(stack + sizeof (struct chx_stack_regs));
  memset (&tp->tc, 0, sizeof (tp->tc));
  tp->tc.reg[REG_SP] = (uint32_t)stack;
  tp->next = tp->prev = tp;
  tp->mutex_list = NULL;
  tp->clp = NULL;
  tp->state = THREAD_EXITED;
  tp->flag_got_cancel = tp->flag_join_req = 0;
  tp->flag_sched_rr = (flags_and_prio & CHOPSTX_SCHED_RR)? 1 : 0;
  tp->flag_detached = (flags_and_prio & CHOPSTX_DETACHED)? 1 : 0;
  tp->prio_orig = tp->prio = prio;
  tp->v = 0;

  chx_cpu_sched_lock ();
  chx_ready_enqueue (tp);
  if (tp->prio > running->prio)
    chx_sched (CHX_YIELD);
  else
    chx_cpu_sched_unlock ();

  return (chopstx_t)tp;
}


/**
 * chopstx_usec_wait_var - Sleep for micro seconds (specified by variable)
 * @var: Pointer to usec
 *
 * Sleep for micro second specified by @var.
 * This is useful to avoid a race condition by making another thread clear
 * @var on condition (to avoid this thread going into sleep).
 */
void
chopstx_usec_wait_var (uint32_t *var)
{
  register uint32_t *usec_p asm ("r8") = var;
  uint32_t usec;
  uint32_t usec0 = 0;

  while (1)
    {
      chx_cpu_sched_lock ();
      if (!usec_p)		/* awakened */
	break;
      *usec_p -= usec0;
      usec = *usec_p;
      if (usec == 0)
	break;
      usec0 = (usec > 200*1000) ? 200*1000: usec;
      if (running->flag_sched_rr)
	chx_timer_dequeue (running);
      chx_spin_lock (&q_timer.lock);
      running->state = THREAD_WAIT_TIME;
      chx_timer_insert (running, usec0);
      chx_spin_unlock (&q_timer.lock);
      chx_sched (CHX_SLEEP);
      asm ("" : "=r" (usec_p) : "r" (usec_p));
    }

  chx_cpu_sched_unlock ();
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
  mutex->q.next = mutex->q.prev = (struct chx_thread *)mutex;
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
	  if (owner->state == THREAD_READY)
	    {
	      ll_prio_enqueue (ll_dequeue (owner), &q_ready);
	      break;
	    }
	  else if (owner->state == THREAD_WAIT_MTX)
	    {
	      m = (chopstx_mutex_t *)owner->v;

	      ll_prio_enqueue (ll_dequeue (owner), m);
	      owner = m->owner;
	      continue;
	    }
	  else if (owner->state == THREAD_WAIT_CND)
	    {
	      chopstx_cond_t *cnd = (chopstx_cond_t *)owner->v;

	      ll_prio_enqueue (ll_dequeue (owner), cnd);
	      break;
	    }
	  else
	    break;
	  /* Assume it's UP no case of: ->state==RUNNING on SMP??? */
	}

      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      ll_prio_enqueue (tp, &mutex->q);
      tp->state = THREAD_WAIT_MTX;
      tp->v = (uint32_t)mutex;
      chx_spin_unlock (&mutex->lock);
      chx_sched (CHX_SLEEP);
    }

  return;
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
  cond->q.next = cond->q.prev = (struct chx_thread *)cond;
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
  ll_prio_enqueue (tp, &cond->q);
  tp->state = THREAD_WAIT_CND;
  tp->v = (uint32_t)cond;
  chx_spin_unlock (&cond->lock);
  chx_sched (CHX_SLEEP);

  if (mutex)
    chopstx_mutex_lock (mutex);
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
  tp = ll_pop (&cond->q);
  if (tp)
    {
      chx_ready_enqueue (tp);
      if (tp->prio > running->prio)
	yield = 1;
    }
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
 * Wake up all thread winting on @cond.
 */
void
chopstx_cond_broadcast (chopstx_cond_t *cond)
{
  struct chx_thread *tp;
  int yield = 0;

  chx_cpu_sched_lock ();
  chx_spin_lock (&cond->lock);
  while ((tp = ll_pop (&cond->q)))
    {
      chx_ready_enqueue (tp);
      if (tp->prio > running->prio)
	yield = 1;
    }
  chx_spin_unlock (&cond->lock);
  if (yield)
    chx_sched (CHX_YIELD);
  else
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
  intr->next = intr_top;
  intr_top = intr;
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
  intr_prev = intr_top;
  for (intr = intr_top; intr; intr = intr->next)
    if (intr == intr0)
      break;

  if (intr == intr_top)
    intr_top = intr_top->next;
  else
    intr_prev->next = intr->next;
  chx_cpu_sched_unlock ();
}


static void
chopstx_release_irq_thread (struct chx_thread *tp)
{
  chopstx_intr_t *intr, *intr_prev;

  chx_cpu_sched_lock ();
  intr_prev = intr_top;
  for (intr = intr_top; intr; intr = intr->next)
    if (intr->tp == tp)
      break;

  if (intr)
    {
      chx_enable_intr (intr->irq_num);
      if (intr == intr_top)
	intr_top = intr_top->next;
      else
	intr_prev->next = intr->next;
    }
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
  chx_cpu_sched_lock ();
  chx_enable_intr (intr->irq_num);
  if (intr->ready == 0)
    {
      if (running->flag_sched_rr)
	chx_timer_dequeue (running);
      running->state = THREAD_WAIT_INT;
      running->v = 0;
      chx_sched (CHX_SLEEP);
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

  chopstx_release_irq_thread (running);
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
      ll_insert (running, &q_join);
      running->v = (uint32_t)tp;
      running->state = THREAD_JOIN;
      chx_spin_unlock (&q_join.lock);
      tp->flag_join_req = 1;
      if (tp->prio < running->prio)
	{
	  tp->prio = running->prio;
	  /*XXX: dequeue and enqueue with new prio.  */
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
 * Canceling the timer, wakup the sleeping thread for it.
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
      tp->tc.reg[REG_EXIT] = 0;
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
 * This function requests a cancellation th the thread @thd.
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
  /* Cancellation points: cond_wait, intr_wait, and usec_wait.  */
  if (tp->state == THREAD_WAIT_CND || tp->state == THREAD_WAIT_INT
      || tp->state == THREAD_WAIT_TIME)
    {
      /* Throw away registers on stack and direct to chopstx_exit.  */
      /* This is pretty much violent, but it works.  */
      p = (struct chx_stack_regs *)tp->tc.reg[REG_SP];
      p->reg[REG_R0] = CHOPSTX_EXIT_CANCELED;
      p->reg[REG_PC] = (uint32_t)chopstx_exit;

      if (tp->state == THREAD_WAIT_CND)
	ll_dequeue (tp);
      else if (tp->state == THREAD_WAIT_TIME)
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
  if (running->flag_got_cancel)
    chopstx_exit ((void *)CHOPSTX_EXIT_CANCELED_IN_SYNC);
}
