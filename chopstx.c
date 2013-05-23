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

/* RUNNING: the current thread. */
struct chx_thread *running;

/* Use this when we support round robin scheduling.  */
#define PREEMPTION_USEC (1000*MHZ) /* 1ms */

/* Double linked list operations.  */
struct chx_dll {
  struct chx_thread *next, *prev;
};


/* READY: priority queue. */
struct chx_ready {
  struct chx_thread *next, *prev;
  struct chx_spinlock lock;
};

static struct chx_ready q_ready;

struct chx_timer {
  struct chx_thread *next, *prev;
  struct chx_spinlock lock;
};

/* threads waiting for timer.  */
static struct chx_timer q_timer;

/* XXX: q_exit; Queue for threads already exited. */

/* Forward declaration(s). */
static void chx_preempt (void);


/**************/
static void chx_LOCK (struct chx_spinlock *lk)
{
  (void)lk;
}

static void chx_UNLOCK (struct chx_spinlock *lk)
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
  uint16_t prio;
  uint16_t prio_orig;
  uint32_t v;
  struct chx_mtx *mutex_list;
} __attribute__((packed));


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
 * Thread status encoded in ->v.
 */
#define THREAD_WAIT_MTX 0x00000001
#define THREAD_WAIT_CND 0x00000002
#define THREAD_WAITTIME 0x00000003

#define THREAD_RUNNING  0x00000000
#define THREAD_WAIT_INT	0x00000004
#define THREAD_EXITED	0x00000008
#define THREAD_READY	0x0000000C

static uint32_t
chx_ready_pop (void)
{
  struct chx_thread *tp;

  chx_LOCK (&q_ready.lock);
  tp = ll_pop (&q_ready);
  if (tp)
    tp->v = THREAD_RUNNING;
  chx_UNLOCK (&q_ready.lock);

  return (uint32_t)tp;
}


static void
chx_ready_push (struct chx_thread *t)
{
  chx_LOCK (&q_ready.lock);
  t->v = THREAD_READY;
  ll_prio_push (t, &q_ready);
  chx_UNLOCK (&q_ready.lock);
}


static void
chx_ready_enqueue (struct chx_thread *t)
{
  chx_LOCK (&q_ready.lock);
  t->v = THREAD_READY;
  ll_prio_enqueue (t, &q_ready);
  chx_UNLOCK (&q_ready.lock);
}

/* Registers on stack (PSP): r0, r1, r2, r3, r12, lr, pc, xpsr */
static void __attribute__ ((naked,used))
sched (void)
{
  register uint32_t r0 asm ("r0");

  asm volatile ("cpsid   i" : : : "memory");

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
		"cpsie   i\n\t"	      /* Unmask interrupts.  */
		"add	r0, #8\n\t"
		"ldm	r0!, {r4, r5, r6, r7}\n\t"
		"ldr	r8, [r0], 4\n\t"
		"ldr	r9, [r0], 4\n\t"
		"ldr	r10, [r0], 4\n\t"
		"ldr	r11, [r0], 4\n\t"
		"ldr	r1, [r0]\n\t"
		"msr	PSP, r1\n\t"
		"mov	r0, #-1\n\t"
		"sub	r0, #2\n\t" /* EXC_RETURN to a thread with PSP */
		"bx	r0\n"
	"3:\n\t"
		"cpsie   i\n\t"	      /* Unmask interrupts.  */
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
		"mov	r0, #-1\n\t"
		"sub	r0, #6\n\t" /* EXC_RETURN to a thread with MSP */
		"bx	r0\n"
		: /* no output */ : "r" (r0) : "memory");
}

void __attribute__ ((naked))
preempt (void)
{
  register uint32_t r0 asm ("r0");

  asm volatile ("ldr	r1, =running\n\t"
		"ldr	r0, [r1]\n\t"
		"cbnz	r0, 0f\n\t"
		/* It's idle which was preempted.  */
		"ldr	r1, =__main_stack_end__\n\t"
		"msr	MSP, r1\n\t"
		"b	sched\n"
	"0:\n\t"
		"ldr	r2, [r0, 48]\n\t" /* Check ->v to avoid RACE.  */
		"cbz	r2, 1f\n\t"
		/* RUNNING is busy on transition, do nothing.  */
		"bx	lr\n"
	"1:\n\t"
		"add	r2, r0, #8\n\t"
		/* Save registers onto CHX_THREAD struct.  */
		"stm	r2!, {r4, r5, r6, r7}\n\t"
		"mov	r3, r8\n\t"
		"mov	r4, r9\n\t"
		"mov	r5, r10\n\t"
		"mov	r6, r11\n\t"
		"mrs	r7, PSP\n\t" /* r13(=SP) in user space.  */
		"stm	r2, {r3, r4, r5, r6, r7}"
		: "=r" (r0): /* no input */ : "memory");

  asm volatile ("cpsid   i" : : : "memory");
  chx_ready_push ((struct chx_thread *)r0);
  asm volatile ("ldr	r1, =running\n\t"
		"mov	r2, #0\n\t"
		"str	r2, [r1]\n\t" /* running := NULL */
		"cpsie   i"	      /* Unmask interrupts.  */
		: /* no output */ : /* no input */ : "memory");

  asm volatile ("b	sched"
		: /* no output */: /* no input */ : "memory");
}


/* system call: sched */
void __attribute__ ((naked))
svc (void)
{
  register uint32_t r0 asm ("r0");
  register uint32_t orig_r0 asm ("r2");

  asm volatile ("ldr	r1, =running\n\t"
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
		"ldr	r2, [r7]"
		: "=r" (r0), "=r" (orig_r0) : /* no input */ : "memory");

  if (orig_r0)
    {
      asm volatile ("cpsid   i" : : : "memory");
      chx_ready_enqueue ((struct chx_thread *)r0);
      asm volatile ("ldr	r1, =running\n\t"
		    "mov	r2, #0\n\t"
		    "str	r2, [r1]\n\t" /* running := NULL */
		    "cpsie	i"	      /* Unmask interrupts.  */
		    : /* no output */ : /* no input */ : "memory");
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
    q->v = (ticks<<8)|THREAD_WAITTIME;
}

static void
chx_timer_insert (struct chx_thread *tp, uint32_t usec)
{
  uint32_t ticks = usec_to_ticks (usec);
  uint32_t next_ticks = *SYST_CVR;
  struct chx_thread *q;

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&q_timer.lock);

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
	  next_ticks = (q->v >> 8);
	}
    }

  if (q == (struct chx_thread *)&q_timer)
    {
      ll_insert (tp, q);
      chx_set_timer (tp->prev, ticks);
      chx_set_timer (tp, 1);	/* Non-zero for the last entry. */
    }
  chx_UNLOCK (&q_timer.lock);
  asm volatile ("cpsie   i" : : : "memory");
}


void
chx_timer_expired (void)
{
  struct chx_thread *t;

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&q_timer.lock);
  if ((t = ll_pop (&q_timer)))
    {
      uint32_t next_tick = t->v >> 8;

      chx_ready_enqueue (t);

      if (!ll_empty (&q_timer))
	{
	  struct chx_thread *t_next;

	  for (t = q_timer.next;
	       t != (struct chx_thread *)&q_timer && next_tick == 0;
	       t = t_next)
	    {
	      next_tick = (t->v >> 8);
	      t_next = t->next;
	      ll_dequeue (t);
	      chx_ready_enqueue (t);
	    }

	  if (!ll_empty (&q_timer))
	    chx_set_timer ((struct chx_thread *)&q_timer, next_tick);
	}
    }

  chx_preempt ();
  chx_UNLOCK (&q_timer.lock);
  asm volatile ("cpsie   i" : : : "memory");
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
}

#define INTR_PRIO (11<<4)

static void
chx_set_intr_prio (uint8_t n)
{
  unsigned int sh = (n & 3) << 3;

  NVIC_IPR (n) = (NVIC_IPR(n) & ~(0xFF << sh)) | (INTR_PRIO << sh);
}

static chopstix_intr_t *intr_top;

void
chx_handle_intr (void)
{
  chopstix_intr_t *intr;
  register uint32_t irq_num;

  asm volatile ("cpsid	i\n\t"
		"mrs	%0, IPSR\n\t"
		"sub	%0, #16"   /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : /* no input */ : "memory");
  chx_disable_intr (irq_num);
  for (intr = intr_top; intr; intr = intr->next)
    if (intr->irq_num == irq_num)
      break;

  if (intr == NULL)
    {				/* Interrupt from unregistered source.  */
      asm volatile ("cpsie   i" : : : "memory");
      return;
    }

  if (intr->t && intr->t->v == THREAD_WAIT_INT)
    {
      intr->ready++;
      chx_ready_enqueue (intr->t);
      chx_preempt ();
    }
  asm volatile ("cpsie   i" : : : "memory");
}

void
chx_systick_init (void)
{
  *SYST_RVR = 0;
  *SYST_CVR = 0;
  *SYST_CSR = 7;
}

static uint32_t *const SHPR3 = (uint32_t *const)0xE000ED20;
#define INTR_PRIO_PENDSV (15<<4)

#define PRIO_DEFAULT 1

void
chx_init (struct chx_thread *tp)
{
  *SHPR3 = (INTR_PRIO_PENDSV << 16);

  memset (&tp->tc, 0, sizeof (tp->tc));
  q_ready.next = q_ready.prev = (struct chx_thread *)&q_ready;
  q_timer.next = q_timer.prev = (struct chx_thread *)&q_timer;
  tp->prio_orig = tp->prio = PRIO_DEFAULT;
  tp->next = tp->prev = tp;
  tp->mutex_list = NULL;
  tp->v = THREAD_RUNNING;

  running = tp;
}


static void
chx_preempt (void)
{
  static volatile uint32_t *const ICSR = (uint32_t *const)0xE000ED04;

  *ICSR = (1 << 28);
  asm volatile ("" : : : "memory");
}

static void
chx_sched (void)
{
  register uint32_t r0 asm ("r0") = 0;

  asm volatile ("svc	#0" : : "r" (r0) : "memory");
}

static void
chx_yield (void)
{
  register uint32_t r0 asm ("r0") = 1;

  asm volatile ("svc	#0" : : "r" (r0) : "memory");
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
  p->reg[REG_LR] = 0;		/* XXX: address of exit??? */
  p->reg[REG_PC] = (uint32_t)thread_entry;
  p->reg[REG_XPSR] = INITIAL_XPSR;

  tp = (struct chx_thread *)(stack + sizeof (struct chx_stack_regs));
  memset (&tp->tc, 0, sizeof (tp->tc));
  tp->prio_orig = tp->prio = attr->prio;
  tp->tc.reg[REG_SP] = (uint32_t)stack;
  tp->next = tp->prev = tp;
  tp->mutex_list = NULL;
  tp->v = THREAD_EXITED;
  *thd = (uint32_t)tp;

  asm volatile ("cpsid   i" : : : "memory");
  chx_ready_enqueue (tp);
  asm volatile ("cpsie   i" : : : "memory");
  if (tp->prio > running->prio)
    chx_yield ();
}


void
chopstx_usleep (uint32_t usec)
{
  while (usec)
    {
      uint32_t usec0 = (usec > 200*1000) ? 200*1000: usec;

      chx_timer_insert (running, usec0);
      chx_sched ();

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
      struct chx_thread *t = running;
      chopstx_mutex_t *m;
      struct chx_thread *owner;

      asm volatile ("cpsid   i" : : : "memory");
      chx_LOCK (&mutex->lock);
      if (mutex->owner == NULL)
	{
	  /* The mutex is acquired.  */
	  mutex->owner = t;
	  mutex->list = t->mutex_list;
	  t->mutex_list = mutex;
	  chx_UNLOCK (&mutex->lock);
	  asm volatile ("cpsie   i" : : : "memory");
	  return;
	}

      m = mutex;
      owner = m->owner;
      while (1)
	{
	  owner->prio = t->prio;
	  if (owner->v == THREAD_READY)
	    {
	      ll_prio_enqueue (ll_dequeue (owner), &q_ready);
	      break;
	    }
	  else if ((owner->v & 0x03) == THREAD_WAIT_MTX)
	    {
	      m = (chopstx_mutex_t *)(owner->v & ~0x03);

	      ll_prio_enqueue (ll_dequeue (owner), m);
	      owner = m->owner;
	      continue;
	    }
	  else if ((owner->v & 0x03) == THREAD_WAIT_CND)
	    {
	      chopstx_cond_t *cnd = (chopstx_cond_t *)(owner->v & ~0x03);

	      ll_prio_enqueue (ll_dequeue (owner), cnd);
	      break;
	    }
	  else
	    break;
	  /* XXX: RUNNING and SMP??? */
	}

      ll_prio_enqueue (t, &mutex->q);
      t->v = (uint32_t)mutex | THREAD_WAIT_MTX;
      chx_UNLOCK (&mutex->lock);
      asm volatile ("cpsie   i" : : : "memory");
      chx_sched ();
    }
}

void
chopstx_mutex_unlock (chopstx_mutex_t *mutex)
{
  struct chx_thread *t;
  int yield = 0;

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&mutex->lock);
  mutex->owner = NULL;
  running->mutex_list = mutex->list;
  mutex->list = NULL;

  t = ll_pop (&mutex->q);
  if (t)
    {
      uint16_t newprio = running->prio_orig;
      chopstx_mutex_t *m;

      chx_ready_enqueue (t);

      /* Examine mutexes we hold, and determine new priority for running.  */
      for (m = running->mutex_list; m; m = m->list)
	if (!ll_empty (&m->q) && m->q.next->prio > newprio)
	  newprio = m->q.next->prio;
      /* Then, assign it.  */
      running->prio = newprio;

      if (t->prio > running->prio)
	yield = 1;
    }

  chx_UNLOCK (&mutex->lock);
  asm volatile ("cpsie   i" : : : "memory");
  if (yield)
    chx_yield ();
}


void
chopstx_cond_init (chopstx_cond_t *cond)
{
  cond->q.next = cond->q.prev = (struct chx_thread *)cond;
}

void
chopstx_cond_wait (chopstx_cond_t *cond, chopstx_mutex_t *mutex)
{
  struct chx_thread *t = running;

  if (mutex)
    chopstx_mutex_unlock (mutex);

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&cond->lock);
  ll_prio_enqueue (t, &cond->q);
  t->v = (uint32_t)cond | THREAD_WAIT_CND;
  chx_UNLOCK (&cond->lock);
  asm volatile ("cpsie   i" : : : "memory");

  chx_sched ();

  if (mutex)
    chopstx_mutex_lock (mutex);
}

void
chopstx_cond_signal (chopstx_cond_t *cond)
{
  struct chx_thread *t;
  int yield = 0;

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&cond->lock);
  t = ll_pop (&cond->q);
  if (t)
    {
      chx_ready_enqueue (t);
      if (t->prio > running->prio)
	yield = 1;
    }
  chx_UNLOCK (&cond->lock);
  asm volatile ("cpsie   i" : : : "memory");

  if (yield)
    chx_yield ();
}

void
chopstx_cond_broadcast (chopstx_cond_t *cond)
{
  struct chx_thread *t;
  int yield = 1;

  asm volatile ("cpsid   i" : : : "memory");
  chx_LOCK (&cond->lock);
  while ((t = ll_pop (&cond->q)))
    {
      chx_ready_enqueue (t);
      if (t->prio > running->prio)
	yield = 1;
    }
  chx_UNLOCK (&cond->lock);
  asm volatile ("cpsie   i" : : : "memory");
  if (yield)
    chx_yield ();
}


void
chopstx_intr_register (chopstix_intr_t *intr, uint8_t irq_num)
{
  chx_disable_intr (irq_num);
  chx_set_intr_prio (irq_num);
  intr->next = intr_top;
  intr_top = intr;
  intr->irq_num = irq_num;
  intr->t = running;
  intr->ready = 0;
}


void
chopstx_wait_intr (chopstix_intr_t *intr)
{
  asm volatile ("cpsid   i" : : : "memory");
  chx_enable_intr (intr->irq_num);
  while (intr->ready == 0)
    {
      intr->t = running;
      running->v = THREAD_WAIT_INT;
      asm volatile ("cpsie   i" : : : "memory");
      chx_sched ();
      asm volatile ("cpsid   i" : : : "memory");
    }
  intr->ready--;
  asm volatile ("cpsie   i" : : : "memory");
}
