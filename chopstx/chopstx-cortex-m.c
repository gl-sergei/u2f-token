/*
 * chopstx-cortex-m.c - Threads and only threads: Arch specific code
 *                      for Cortex-M0/M3
 *
 * Copyright (C) 2013, 2014, 2015, 2016, 2017
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
static volatile uint32_t *const SYST_CSR = (uint32_t *)0xE000E010;
static volatile uint32_t *const SYST_RVR = (uint32_t *)0xE000E014;
static volatile uint32_t *const SYST_CVR = (uint32_t *)0xE000E018;

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

static struct NVIC *const NVIC = (struct NVIC *)0xE000E100;
#define NVIC_ISER(n)	(NVIC->ISER[n >> 5])
#define NVIC_ICER(n)	(NVIC->ICER[n >> 5])
#define NVIC_ICPR(n)	(NVIC->ICPR[n >> 5])
#define NVIC_IPR(n)	(NVIC->IPR[n >> 2])


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

static volatile uint32_t *const ICSR = (uint32_t *)0xE000ED04;

/* Priority control.  */
static uint32_t *const AIRCR = (uint32_t *)0xE000ED0C;
static uint32_t *const SHPR2 = (uint32_t *)0xE000ED1C;
static uint32_t *const SHPR3 = (uint32_t *)0xE000ED20;

static void
chx_prio_init (void)
{
  *AIRCR = 0x05FA0000 | ( 5 << 8); /* PRIGROUP = 5, 2-bit:2-bit. */
  *SHPR2 = (CPU_EXCEPTION_PRIORITY_SVC << 24);
  *SHPR3 = ((CPU_EXCEPTION_PRIORITY_SYSTICK << 24)
	    | (CPU_EXCEPTION_PRIORITY_PENDSV << 16));
}


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


void
chx_handle_intr (void)
{
  struct chx_pq *p;
  register uint32_t irq_num;

  asm volatile ("mrs	%0, IPSR\n\t"
		"sub	%0, #16"   /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : /* no input */ : "memory");

  chx_disable_intr (irq_num);
  chx_spin_lock (&q_intr.lock);
  for (p = q_intr.q.next; p != (struct chx_pq *)&q_intr.q; p = p->next)
    if (p->v == irq_num)
      {			/* should be one at most. */
	struct chx_px *px = (struct chx_px *)p;

	ll_dequeue (p);
	chx_wakeup (p);
	chx_request_preemption (px->master->prio);
	break;
      }
  chx_spin_unlock (&q_intr.lock);
}

static void
chx_init_arch (struct chx_thread *tp)
{
  memset (&tp->tc, 0, sizeof (tp->tc));
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
 * Returns:
 *          1 on wakeup by others.
 *          0 on normal wakeup.
 *         -1 on cancellation.
 */
static uintptr_t __attribute__ ((naked, noinline))
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
       : /* no input */
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
		"add	r0, #16\n\t" /* ->V */
		"ldr	r1, [r0]\n\t"
		"str	r1, [sp]\n\t"
		/**/
		"add	r0, #4\n\t"
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
		"ldr	r2, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r2, r1\n\t"	/* Ensure Thumb-mode */
		"str	r2, [sp, #32]\n\t"
		"msr	APSR_nzcvq, r0\n\t"
		/**/
		"ldr	r0, [sp, #20]\n\t"
		"mov	lr, r0\n\t"
		"ldr	r0, [sp, #16]\n\t"
		"mov	r12, r0\n\t"
		"pop	{r0, r1, r2, r3}\n\t"
		"add	sp, #16\n\t"
		"pop	{pc}\n"
	"2:\n\t"
		"ldr	r2, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r2, r1\n\t"	/* Ensure Thumb-mode */
		"str	r2, [sp, #28]\n\t"
		"msr	APSR_nzcvq, r0\n\t"
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

  return (uintptr_t)tp;
}

extern void cause_link_time_error_unexpected_size_of_struct_chx_thread (void);

static struct chx_thread *
chopstx_create_arch (uintptr_t stack_addr, size_t stack_size,
		     voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  void *stack;
  struct chx_stack_regs *p;

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

  return tp;
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
	: "0" (CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED)
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
	"cbz	r0, 0f\n\t"
	"ldr	r1, [r0, #16]\n\t" /* ->V */
	"str	r1, [sp]\n\t"
    "0:\n\t"
	"b	.L_CONTEXT_SWITCH"
	: /* no output */ : "r" (tp) : "memory");
}
#endif
