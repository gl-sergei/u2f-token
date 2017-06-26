/*
 * entry.c - Entry routine when reset and interrupt vectors.
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

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include "board.h"

#ifdef GNU_LINUX_EMULATION
int emulated_main (int, const char **);
void chx_init (struct chx_thread *);
void chx_systick_init (void);
extern struct chx_thread main_thread;

int
main (int argc, const char *argv[])
{
  chx_init (&main_thread);
  chx_systick_init ();
  emulated_main (argc, argv);
}
#else
#if defined(USE_SYS3) || defined(USE_SYS_CLOCK_GPIO_SETTING)
#define REQUIRE_CLOCK_GPIO_SETTING_IN_SYS
#include "sys.h"
/*
 * Avoid medium density specific code and prepare for high density
 * device, too.
 */
#undef STM32F10X_MD
#else
#if defined (MCU_KINETIS_L)
#include "mcu/clk_gpio_init-mkl27z.c"
#else
#include "mcu/clk_gpio_init-stm32.c"
#endif
#endif


#ifdef MAKE_ENTRY_PUBLIC
#define STATIC_ENTRY
#else
#define STATIC_ENTRY static
#endif

extern uint8_t __main_stack_end__;
#if defined(__ARM_ARCH_7M__)
extern void svc (void);
#endif
extern void preempt (void);
extern void chx_timer_expired (void);
extern void chx_handle_intr (void);

static void nmi (void)
{
  for (;;);
}

static void hard_fault (void)
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

#define C_S_SUB(arg0, arg1, arg2) arg0 #arg1 arg2
#define COMPOSE_STATEMENT(arg0,arg1,arg2)  C_S_SUB (arg0, arg1, arg2)

#if defined(__ARM_ARCH_6M__)
__attribute__ ((used,section(".bss.startup.0")))
uint32_t vectors_in_ram[48];
#endif

/*
 * This routine only changes PSP and not MSP.
 */
STATIC_ENTRY __attribute__ ((naked,section(".text.startup.0")))
void
entry (void)
{
  asm volatile ("bl	clock_init\n\t"
		/* Clear BSS section.  */
		"mov	r0, #0\n\t"
		"ldr	r1, =_bss_start\n\t"
		"ldr	r2, =_bss_end\n"
	"0:\n\t"
		"cmp	r1, r2\n\t"
		"beq	1f\n\t"
#if defined(__ARM_ARCH_6M__)
		"str	r0, [r1]\n\t"
		"add	r1, #4\n\t"
#else
		"str	r0, [r1], #4\n\t"
#endif
		"b	0b\n"
	"1:\n\t"
		/* Copy data section.  */
		"ldr	r1, =_data\n\t"
		"ldr	r2, =_edata\n\t"
		"ldr	r3, =_textdata\n"
	"2:\n\t"
		"cmp	r1, r2\n\t"
		"beq	3f\n\t"
#if defined(__ARM_ARCH_6M__)
		"ldr	r0, [r3]\n\t"
		"str	r0, [r1]\n\t"
		"add	r3, #4\n\t"
		"add	r1, #4\n\t"
#else
		"ldr	r0, [r3], #4\n\t"
		"str	r0, [r1], #4\n\t"
#endif
		"b	2b\n"
	"3:\n\t"
		/* Switch to PSP.  */
		"ldr	r0, =__process0_stack_end__\n\t"
		COMPOSE_STATEMENT ("sub	r0, #", CHOPSTX_THREAD_SIZE, "\n\t")
		"msr	PSP, r0\n\t" /* Process (main routine) stack.  */
		"mov	r1, #2\n\t"
		"msr	CONTROL, r1\n\t"
		"isb\n\t"
		"bl	chx_init\n\t"
		"bl	chx_systick_init\n\t"
		"bl	gpio_init\n\t"
		/* Enable interrupts.  */
#if defined(__ARM_ARCH_7M__)
		"mov	r0, #0\n\t"
		"msr	BASEPRI, r0\n\t"
#endif
		"cpsie	i\n\t"
		/* Call main.  */
		"mov	r1, r0\n\t"
		"bl	main\n"
	"4:\n\t"
		"b	4b"
		: /* no output */ : /* no input */ : "memory");
}


typedef void (*handler)(void);

handler vector_table[] __attribute__ ((section(".startup.vectors"))) = {
  (handler)(&__main_stack_end__ - 32),
  entry,
  nmi,		/* nmi */
  hard_fault,		/* hard fault */
  /* 0x10 */
  mem_manage,		/* mem manage */
  bus_fault,		/* bus fault */
  usage_fault,		/* usage fault */
  none,
  /* 0x20 */
  none, none, none,		/* reserved */
#if defined(__ARM_ARCH_6M__)
  none,				/* SVCall */
#elif defined(__ARM_ARCH_7M__)
  svc,				/* SVCall */
#endif  
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
#if !defined(__ARM_ARCH_6M__)
  /* STM32F0 doesn't have more.  */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
#endif
#if !defined(STM32F10X_MD)
  /* High-density chips have more; RTCAlarm, USBWakeup, ... , DMA2_Channel4_5 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
#endif
};
#endif
