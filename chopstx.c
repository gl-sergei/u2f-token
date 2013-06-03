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
 * Note: Lower has higher precedence.
 *
 * Prio  0: svc
 * Prio 32: thread temporarily inhibiting schedule for critical region
 * Prio 64: systick, external interrupt
 * Prio 96: pendsv
 */

#define CPU_EXCEPTION_PRIORITY_SVC           0
#define CPU_EXCEPTION_PRIORITY_CLEAR         0
#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 32
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     64
#define CPU_EXCEPTION_PRIORITY_PENDSV        96

static void
chx_cpu_sched_lock (void)
{
  register uint32_t tmp = CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED;
  asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
}

static void
chx_cpu_sched_unlock (void)
{
  register uint32_t tmp = CPU_EXCEPTION_PRIORITY_CLEAR;
  asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
}

void __attribute__((weak, noreturn))
chx_fatal (uint32_t err_code)
{
  (void)err_code;
  for (;;);
}


/* RUNNING: the current thread. */
struct chx_thread *running;

/* Use this when we support round robin scheduling.  */
#define PREEMPTION_USEC (1000*MHZ) /* 1ms */

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
};


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
  struct chx_thread *tp0 = tp;

  tp->next->prev = tp->prev;
  tp->prev->next = tp->next;
  return tp0;
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


static uint32_t
chx_ready_pop (void)
{
  struct chx_thread *tp;

  chx_spin_lock (&q_ready.lock);
  tp = ll_pop (&q_ready);
  if (tp)
    tp->state = THREAD_RUNNING;
  chx_spin_unlock (&q_ready.lock);

  return (uint32_t)tp;
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
static void __attribute__ ((naked,used))
sched (void)
{
  register uint32_t r0 asm ("r0");

  r0 = chx_ready_pop ();

  asm volatile (/* Now, r0 points to the thread to be switched.  */
		/* Put it to *running.  */
		"ldr	r1, =running\n\t"
		/* Update running.  */
		"str	r0, [r1]\n\t"
		"cbz	r0, 3f\n\t"
		/**/
		"str	r0, [r0]\n\t"
		"str	r0, [r0, 4]\n\t"
		"add	r0, #8\n\t"
		"ldm	r0!, {r4, r5, r6, r7}\n\t"
		"ldr	r8, [r0], 4\n\t"
		"ldr	r9, [r0], 4\n\t"
		"ldr	r10, [r0], 4\n\t"
		"ldr	r11, [r0], 4\n\t"
		"ldr	r1, [r0]\n\t"
		"msr	PSP, r1\n\t"
		/**/
		"mov	r0, #-1\n\t"
		"sub	r0, #2\n\t" /* EXC_RETURN to a thread with PSP */
		"bx	r0\n"
	"3:\n\t"
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
		"mov	r0, #-1\n\t"
		"sub	r0, #6\n\t" /* EXC_RETURN to a thread with MSP */
		"bx	r0\n"
		: /* no output */ : "r" (r0) : "memory");
}

void __attribute__ ((naked))
preempt (void)
{
  register uint32_t r0 asm ("r0");

  asm ("ldr	r1, =running\n\t"
       "ldr	r0, [r1]\n\t"
       "cbnz	r0, 0f\n\t"
       /* It's idle which was preempted.  */
       "mov	r0, #2\n\t"
       "msr	BASEPRI, r0\n\t"	      /* mask any interrupts.  */
       "ldr	r1, =__main_stack_end__\n\t"
       "msr	MSP, r1\n\t"
       "b	sched\n"
  "0:\n\t"
       "add	r2, r0, #8\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r2!, {r4, r5, r6, r7}\n\t"
       "mov	r3, r8\n\t"
       "mov	r4, r9\n\t"
       "mov	r5, r10\n\t"
       "mov	r6, r11\n\t"
       "mrs	r7, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r2, {r3, r4, r5, r6, r7}"
       : "=r" (r0)
       : /* no input */
       : "r1", "r2", "r3", "r4", "r7", "cc", "memory");

  chx_cpu_sched_lock ();
  chx_ready_push ((struct chx_thread *)r0);
  running = NULL;

  asm volatile ("b	sched"
		: /* no output */: /* no input */ : "memory");
}


/* system call: sched */
void __attribute__ ((naked))
svc (void)
{
  register uint32_t r0 asm ("r0");
  register uint32_t orig_r0 asm ("r1");

  asm ("ldr	r1, =running\n\t"
       "ldr	r0, [r1]\n\t"
       "add	r2, r0, #8\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r2!, {r4, r5, r6, r7}\n\t"
       "mov	r3, r8\n\t"
       "mov	r4, r9\n\t"
       "mov	r5, r10\n\t"
       "mov	r6, r11\n\t"
       "mrs	r7, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r2, {r3, r4, r5, r6, r7}\n\t"
       "ldr	r1, [r7]"
       : "=r" (r0), "=r" (orig_r0)
       : /* no input */
       : "r2", "r3", "r4", "r7", "memory");

  if (orig_r0)
    {
      chx_ready_enqueue ((struct chx_thread *)r0);
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
    {
      q->state = THREAD_WAIT_TIME;
      q->v = ticks;
    }
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

  chx_cpu_sched_lock ();
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
  chx_cpu_sched_unlock ();
}


void
chx_timer_expired (void)
{
  struct chx_thread *tp;
  chopstx_prio_t prio = 0;

  chx_cpu_sched_lock ();
  chx_spin_lock (&q_timer.lock);
  if ((tp = ll_pop (&q_timer)))
    {
      uint32_t next_tick = tp->v;

      chx_ready_enqueue (tp);

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
	      if (tp->prio > prio)
		prio = tp->prio;
	    }

	  if (!ll_empty (&q_timer))
	    chx_set_timer ((struct chx_thread *)&q_timer, next_tick);
	}
    }

  if (running == NULL || running->prio < prio)
    chx_request_preemption ();
  chx_spin_unlock (&q_timer.lock);
  chx_cpu_sched_unlock ();
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

  chx_cpu_sched_lock ();
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
  chx_cpu_sched_unlock ();
}

void
chx_systick_init (void)
{
  *SYST_RVR = 0;
  *SYST_CVR = 0;
  *SYST_CSR = 7;
}

static uint32_t *const SHPR2 = (uint32_t *const)0xE000ED1C;
static uint32_t *const SHPR3 = (uint32_t *const)0xE000ED20;

#define PRIO_DEFAULT 1

void
chx_init (struct chx_thread *tp)
{
  *SHPR2 = (CPU_EXCEPTION_PRIORITY_SVC << 24); /* SVCall */
  *SHPR3 = ((CPU_EXCEPTION_PRIORITY_INTERRUPT << 24) /* SysTick */
	    | (CPU_EXCEPTION_PRIORITY_PENDSV << 16)); /* PendSV */

  memset (&tp->tc, 0, sizeof (tp->tc));
  q_ready.next = q_ready.prev = (struct chx_thread *)&q_ready;
  q_timer.next = q_timer.prev = (struct chx_thread *)&q_timer;
  q_exit.next = q_exit.prev = (struct chx_thread *)&q_exit;
  q_join.next = q_join.prev = (struct chx_thread *)&q_join;
  tp->next = tp->prev = tp;
  tp->mutex_list = NULL;
  tp->state = THREAD_RUNNING;
  tp->flag_detached = tp->flag_got_cancel
    = tp->flag_join_req = tp->flag_sched_rr = 0;
  tp->prio_orig = tp->prio = PRIO_DEFAULT;
  tp->v = 0;

  running = tp;
}


static void
chx_request_preemption (void)
{
  *ICSR = (1 << 28);
  asm volatile ("" : : : "memory");
}

static void
chx_sched (void)
{
  register uint32_t r0 asm ("r0") = 0;

  asm volatile ("svc	#0" : : "r" (r0): "memory");
}

static void
chx_yield (void)
{
  register uint32_t r0 asm ("r0") = 1;

  asm volatile ("svc	#0" : : "r" (r0) : "memory");
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

  chx_spin_lock (&q_exit.lock);
  if (running->flag_detached)
    running->state = THREAD_FINISHED;
  else
    {
      ll_insert (running, &q_exit);
      running->state = THREAD_EXITED;
    }
  chx_spin_unlock (&q_exit.lock);
  asm volatile ("" : : "r" (r8) : "memory");
  chx_sched ();
  /* never comes here. */
  chx_cpu_sched_unlock ();
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

void
chopstx_attr_init (chopstx_attr_t *attr)
{
  attr->prio = PRIO_DEFAULT;
  attr->addr = 0;
  attr->size = 0;
}

void
chopstx_attr_setschedparam (chopstx_attr_t *attr, uint8_t prio)
{
  attr->prio = prio;
}

void
chopstx_attr_setstack (chopstx_attr_t *attr, uint32_t addr, size_t size)
{
  attr->addr = addr;
  attr->size = size;
}


void
chopstx_create (chopstx_t *thd, const chopstx_attr_t *attr,
		void *(thread_entry) (void *), void *arg)
{
  struct chx_thread *tp;
  void *stack;
  struct chx_stack_regs *p;

  if (attr->size < sizeof (struct chx_thread) + 8 * sizeof (uint32_t))
    return;
  
  stack = (void *)(attr->addr + attr->size - sizeof (struct chx_thread)
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
  tp->state = THREAD_EXITED;
  tp->flag_detached = tp->flag_got_cancel
    = tp->flag_join_req = tp->flag_sched_rr = 0;
  tp->prio_orig = tp->prio = attr->prio;
  tp->v = 0;
  *thd = (uint32_t)tp;

  chx_cpu_sched_lock ();
  chx_ready_enqueue (tp);
  if (tp->prio > running->prio)
    chx_yield ();
  chx_cpu_sched_unlock ();
}


void
chopstx_usec_wait (uint32_t usec)
{
  while (usec)
    {
      uint32_t usec0 = (usec > 200*1000) ? 200*1000: usec;

      chx_cpu_sched_lock ();
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (running, usec0);
      chx_spin_unlock (&q_timer.lock);
      chx_sched ();
      chx_cpu_sched_unlock ();
      usec -= usec0;
    }
}


void
chopstx_mutex_init (chopstx_mutex_t *mutex)
{
  mutex->q.next = mutex->q.prev = (struct chx_thread *)mutex;
  mutex->list = NULL;
}


void
chopstx_mutex_lock (chopstx_mutex_t *mutex)
{
  while (1)
    {
      struct chx_thread *tp = running;
      chopstx_mutex_t *m;
      struct chx_thread *owner;

      chx_cpu_sched_lock ();
      chx_spin_lock (&mutex->lock);
      if (mutex->owner == NULL)
	{
	  /* The mutex is acquired.  */
	  mutex->owner = tp;
	  mutex->list = tp->mutex_list;
	  tp->mutex_list = mutex;
	  chx_spin_unlock (&mutex->lock);
	  chx_cpu_sched_unlock ();
	  break;
	}

      m = mutex;
      owner = m->owner;
      while (1)
	{
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

      ll_prio_enqueue (tp, &mutex->q);
      tp->state = THREAD_WAIT_MTX;
      tp->v = (uint32_t)mutex;
      chx_spin_unlock (&mutex->lock);
      chx_sched ();
      chx_cpu_sched_unlock ();
    }

  return;
}


void
chopstx_mutex_unlock (chopstx_mutex_t *mutex)
{
  chopstx_prio_t prio;

  chx_cpu_sched_lock ();
  chx_spin_lock (&mutex->lock);
  prio = chx_mutex_unlock (mutex);
  chx_spin_unlock (&mutex->lock);
  if (prio > running->prio)
    chx_yield ();
  chx_cpu_sched_unlock ();
}


void
chopstx_cond_init (chopstx_cond_t *cond)
{
  cond->q.next = cond->q.prev = (struct chx_thread *)cond;
}


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

  chx_spin_lock (&cond->lock);
  ll_prio_enqueue (tp, &cond->q);
  tp->state = THREAD_WAIT_CND;
  tp->v = (uint32_t)cond;
  chx_spin_unlock (&cond->lock);
  chx_sched ();
  chx_cpu_sched_unlock ();

  if (mutex)
    chopstx_mutex_lock (mutex);
}


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
    chx_yield ();
  chx_cpu_sched_unlock ();
}


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
    chx_yield ();
  chx_cpu_sched_unlock ();
}


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


void
chopstx_intr_wait (chopstx_intr_t *intr)
{
  chx_cpu_sched_lock ();
  chx_enable_intr (intr->irq_num);
  if (intr->ready == 0)
    {
      running->state = THREAD_WAIT_INT;
      running->v = 0;
      chx_sched ();
    }
  intr->ready--;
  chx_cpu_sched_unlock ();
}


/* The RETVAL is saved into register R8.  */
void __attribute__((noreturn))
chopstx_exit (void *retval)
{
  struct chx_mtx *m, *m_next;

  /* XXX: Call registered "release" routines here.  */

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


void
chopstx_join (chopstx_t thd, void **ret)
{
  struct chx_thread *tp = (struct chx_thread *)thd;

  /* XXX: check if another thread is waiting same thread and call fatal. */
  /* XXX: dead lock detection (waiting each other) and call fatal. */

  chx_cpu_sched_lock ();
  if (tp->flag_detached)
    {
      chx_cpu_sched_unlock ();
      chx_fatal (CHOPSTX_ERR_JOIN);
    }

  if (tp->state != THREAD_EXITED)
    {
      chx_spin_lock (&q_join.lock);
      ll_insert (running, &q_join);
      running->v = (uint32_t)tp;
      running->state = THREAD_JOIN;
      chx_spin_unlock (&q_join.lock);
      tp->flag_join_req = 1;
      if (tp->prio < running->prio)
	tp->prio = running->prio;
      chx_sched ();
    }

  tp->state = THREAD_FINISHED;
  if (ret)
    *ret = (void *)tp->tc.reg[REG_EXIT]; /* R8 */
  chx_cpu_sched_unlock ();
}


void
chopstx_cancel (chopstx_t thd)
{
  struct chx_thread *tp = (struct chx_thread *)thd;
  struct chx_stack_regs *p;
  int yield = 0;

  /* Assume it's UP no case of: tp->state==RUNNING on SMP??? */
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
    chx_yield ();
  chx_cpu_sched_unlock ();
}


void
chopstx_testcancel (void)
{
  if (running->flag_got_cancel)
    chopstx_exit ((void *)CHOPSTX_EXIT_CANCELED_IN_SYNC);
}
