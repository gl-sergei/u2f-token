/*
 * reset.c - No system routines, but only RESET handler for STM32F030.
 *
 * Copyright (C) 2015 Flying Stone Technology
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
  chx_handle_intr /* WWDG */,     chx_handle_intr /* PVD */,
  chx_handle_intr /* TAMPER */,   chx_handle_intr /* RTC */,
  chx_handle_intr /* FLASH */,    chx_handle_intr /* RCC */,
  chx_handle_intr /* EXTI0 */,    chx_handle_intr /* EXTI1 */,
  /* 0x60 */
  chx_handle_intr /* EXTI2 */,    chx_handle_intr /* EXTI3 */,
  chx_handle_intr /* EXTI4 */,    chx_handle_intr /* DMA1 CH1 */,
  chx_handle_intr /* DMA1 CH2 */, chx_handle_intr /* DMA1 CH3 */,
  chx_handle_intr /* DMA1 CH4 */, chx_handle_intr /* DMA1 CH5 */,
  /* 0x80 */
  chx_handle_intr /* DMA1 CH6 */, chx_handle_intr /* DMA1 CH7 */,
  chx_handle_intr /* ADC1_2 */,   chx_handle_intr /* USB HP */,
  /* 0x90 */
  chx_handle_intr /* USB LP */,   chx_handle_intr /* CAN */, 
  /* ... and more.  EXT9_5, TIMx, I2C, SPI, USART, EXT15_10 */
  chx_handle_intr,                chx_handle_intr,
  /* 0xA0 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  /* 0xc0 */
};
