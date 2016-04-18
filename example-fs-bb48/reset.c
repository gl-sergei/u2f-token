/*
 * sys.c - No system routines, but only RESET handler for MKL27Z256.
 *
 * Copyright (C) 2016 Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty provided the copyright
 * notice and this notice are preserved.  This file is offered as-is,
 * without any warranty.
 *
 */

#include <stdint.h>
#include <stdlib.h>

static void __attribute__ ((naked))
reset (void)
{
  asm volatile ("cpsid	i\n\t"		/* Mask all interrupts. */
		"mov	r0, pc\n\t"	/* r0 = PC & ~0x0fff */
		"mov	r1, #0x10\n\t"
		"lsl	r1, #8\n\t"
		"sub	r1, r1, #1\n\t"
		"bic	r0, r0, r1\n\t"
		"ldr	r2, [r0]\n\t"
		"msr	MSP, r2\n\t"	/* Main (exception handler) stack. */
		"b	entry\n\t"
		: /* no output */ : /* no input */ : "memory");
  /* Never reach here. */
}

extern uint8_t __main_stack_end__;
extern void preempt (void);
extern void chx_timer_expired (void);
extern void chx_handle_intr (void);

static void nmi (void)
{
  for (;;);
}

static void __attribute__ ((naked))
hard_fault (void)
{
  for (;;);
}

static void mem_manage (void)
{
  for (;;);
}

static void bus_fault (void)
{
  for (;;);
}

static void usage_fault (void)
{
  for (;;);
}

static void none (void)
{
}


typedef void (*handler)(void);
extern uint8_t __main_stack_end__;

handler vector[] __attribute__ ((section(".vectors"))) = {
  (handler)(&__main_stack_end__ - 32),
  reset,
  nmi,		/* nmi */
  hard_fault,		/* hard fault */
  /* 0x10 */
  mem_manage,		/* mem manage */
  bus_fault,		/* bus fault */
  usage_fault,		/* usage fault */
  none,
  /* 0x20 */
  none, none, none,		/* reserved */
  none,				/* SVCall */
  none,				/* Debug */
  none,				/* reserved */
  preempt,			/* PendSV */
  chx_timer_expired,		/* SysTick */
  /* 0x40 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0x60 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0x80 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0xA0 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0xc0 */
};

uint32_t flash_config[] __attribute__ ((section(".flash_config"))) = {
  0xffffffff, 0xffffffff, /* Backdoor comparison key. */
  0xffffffff, /* Protection bytes */
  0xffff3ffe, /* FSEC=0xfe, FOPT=0x3f */
  /* FOPT=0x3f:
   * BOOTSRC_SEL=00: Boot from flash
   */
  /* FSEC=0xfe:
   * unsecure
   */
};
