/*
 * entry.c - Entry routine when reset and interrupt vectors.
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
#include <chopstx.h>

#ifdef HAVE_SYS_H
#define INLINE __attribute__ ((used))
#include "sys.h"
#else
#include "board.h"

#define STM32_SW_PLL		(2 << 0)
#define STM32_PLLSRC_HSE	(1 << 16)

#define STM32_PLLXTPRE_DIV1	(0 << 17)
#define STM32_PLLXTPRE_DIV2	(1 << 17)

#define STM32_HPRE_DIV1		(0 << 4)

#define STM32_PPRE1_DIV2	(4 << 8)

#define STM32_PPRE2_DIV1        (0 << 11)
#define STM32_PPRE2_DIV2	(4 << 11)

#define STM32_ADCPRE_DIV4	(1 << 14)
#define STM32_ADCPRE_DIV6       (2 << 14)

#define STM32_USBPRE_DIV1P5     (0 << 22)

#define STM32_MCO_NOCLOCK	(0 << 24)

#define STM32_SW		STM32_SW_PLL
#define STM32_PLLSRC		STM32_PLLSRC_HSE
#define STM32_HPRE		STM32_HPRE_DIV1
#define STM32_PPRE1		STM32_PPRE1_DIV2
#define STM32_PPRE2		STM32_PPRE2_DIV1
#define STM32_ADCPRE		STM32_ADCPRE_DIV6
#define STM32_MCOSEL		STM32_MCO_NOCLOCK
#define STM32_USBPRE            STM32_USBPRE_DIV1P5

#define STM32_PLLCLKIN		(STM32_HSECLK / 1)
#define STM32_PLLMUL		((STM32_PLLMUL_VALUE - 2) << 18)
#define STM32_PLLCLKOUT		(STM32_PLLCLKIN * STM32_PLLMUL_VALUE)
#define STM32_SYSCLK		STM32_PLLCLKOUT
#define STM32_HCLK		(STM32_SYSCLK / 1)

#define STM32_FLASHBITS		0x00000012

#define PERIPH_BASE	0x40000000
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)

struct RCC {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
};

#define RCC_BASE		(AHBPERIPH_BASE + 0x1000)
static struct RCC *const RCC = ((struct RCC *const)RCC_BASE);

#define RCC_APB1ENR_USBEN	0x00800000
#define RCC_APB1RSTR_USBRST	0x00800000

#define RCC_CR_HSION		0x00000001
#define RCC_CR_HSIRDY		0x00000002
#define RCC_CR_HSITRIM		0x000000F8
#define RCC_CR_HSEON		0x00010000
#define RCC_CR_HSERDY		0x00020000
#define RCC_CR_PLLON		0x01000000
#define RCC_CR_PLLRDY		0x02000000

#define RCC_CFGR_SWS		0x0000000C
#define RCC_CFGR_SWS_HSI	0x00000000

#define RCC_AHBENR_CRCEN        0x0040

struct FLASH {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
};

#define FLASH_R_BASE	(AHBPERIPH_BASE + 0x2000)
static struct FLASH *const FLASH = ((struct FLASH *const) FLASH_R_BASE);

static void __attribute__((used))
clock_init (void)
{
  /* HSI setup */
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;
  RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION;
  RCC->CFGR = 0;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;

  /* HSE setup */
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;

  /* PLL setup */
  RCC->CFGR |= STM32_PLLMUL | STM32_PLLXTPRE | STM32_PLLSRC;
  RCC->CR   |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  /* Clock settings */
  RCC->CFGR = STM32_MCOSEL | STM32_USBPRE | STM32_PLLMUL | STM32_PLLXTPRE
    | STM32_PLLSRC | STM32_ADCPRE | STM32_PPRE2 | STM32_PPRE1 | STM32_HPRE;

  /* Flash setup */
  FLASH->ACR = STM32_FLASHBITS;

  /* CRC */
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  /* Switching on the configured clock source. */
  RCC->CFGR |= STM32_SW;
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;
}

#define RCC_APB2RSTR_AFIORST	0x00000001
#define RCC_APB2RSTR_IOPARST	0x00000004
#define RCC_APB2RSTR_IOPBRST	0x00000008
#define RCC_APB2RSTR_IOPCRST	0x00000010
#define RCC_APB2RSTR_IOPDRST	0x00000020

#define RCC_APB2ENR_AFIOEN	0x00000001
#define RCC_APB2ENR_IOPAEN	0x00000004
#define RCC_APB2ENR_IOPBEN	0x00000008
#define RCC_APB2ENR_IOPCEN	0x00000010
#define RCC_APB2ENR_IOPDEN	0x00000020


struct AFIO
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;
};

#define AFIO_BASE 0x40010000
static struct AFIO *const AFIO = (struct AFIO *const)AFIO_BASE;

#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP 0x00000800
#define AFIO_MAPR_SWJ_CFG_DISABLE         0x04000000


struct GPIO {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
};

#define GPIOA_BASE	(APB2PERIPH_BASE + 0x0800)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE	(APB2PERIPH_BASE + 0x0C00)
#define GPIOB		((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE	(APB2PERIPH_BASE + 0x1000)
#define GPIOC		((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE	(APB2PERIPH_BASE + 0x1400)
#define GPIOD		((struct GPIO *) GPIOD_BASE)
#define GPIOE_BASE	(APB2PERIPH_BASE + 0x1800)
#define GPIOE		((struct GPIO *) GPIOE_BASE)

static struct GPIO *const GPIO_USB = ((struct GPIO *const) GPIO_USB_BASE);
static struct GPIO *const GPIO_LED = ((struct GPIO *const) GPIO_LED_BASE);
#ifdef GPIO_OTHER_BASE
static struct GPIO *const GPIO_OTHER = ((struct GPIO *const) GPIO_OTHER_BASE);
#endif

static void __attribute__((used))
gpio_init (void)
{
  /* Enable GPIO clock. */
  RCC->APB2ENR |= RCC_APB2ENR_IOP_EN;
  RCC->APB2RSTR = RCC_APB2RSTR_IOP_RST;
  RCC->APB2RSTR = 0;

#ifdef AFIO_MAPR_SOMETHING
  AFIO->MAPR |= AFIO_MAPR_SOMETHING;
#endif

  GPIO_USB->ODR = VAL_GPIO_ODR;
  GPIO_USB->CRH = VAL_GPIO_CRH;
  GPIO_USB->CRL = VAL_GPIO_CRL;

#if GPIO_USB_BASE != GPIO_LED_BASE
  GPIO_LED->ODR = VAL_GPIO_LED_ODR;
  GPIO_LED->CRH = VAL_GPIO_LED_CRH;
  GPIO_LED->CRL = VAL_GPIO_LED_CRL;
#endif

#ifdef GPIO_OTHER_BASE
  GPIO_OTHER->ODR = VAL_GPIO_OTHER_ODR;
  GPIO_OTHER->CRH = VAL_GPIO_OTHER_CRH;
  GPIO_OTHER->CRL = VAL_GPIO_OTHER_CRL;
#endif
}
#endif


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

/*
 * This routine only changes PSP and not MSP.
 */
static __attribute__ ((naked,section(".text.startup.0")))
void entry (void)
{
  asm volatile ("bl	clock_init\n\t"
		/* Clear BSS section.  */
		"mov	r0, #0\n\t"
		"ldr	r1, =_bss_start\n\t"
		"ldr	r2, =_bss_end\n"
	"0:\n\t"
		"cmp	r1, r2\n\t"
		"beq	1f\n\t"
		"str	r0, [r1], #4\n\t"
		"b	0b\n"
	"1:\n\t"
		/* Copy data section.  */
		"ldr	r1, =_data\n\t"
		"ldr	r2, =_edata\n\t"
		"ldr	r3, =_textdata\n"
	"2:\n\t"
		"cmp	r1, r2\n\t"
		"beq	3f\n\t"
		"ldr	r0, [r3], #4\n\t"
		"str	r0, [r1], #4\n\t"
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
		"mov	r0, #0\n\t"
		"msr	BASEPRI, r0\n\t"
		"cpsie	i\n\t"
		/* Call main.  */
		"mov	r1, r0\n\t"
		"bl	main\n"
	"4:\n\t"
		"b	4b"
		: /* no output */ : /* no input */ : "memory");
}


typedef void (*handler)(void);
extern uint8_t __main_stack_end__;

extern void svc (void);
extern void preempt (void);
extern void chx_timer_expired (void);
extern void chx_handle_intr (void);

handler vector_table[] __attribute__ ((section(".startup.vectors"))) = {
  (handler)&__main_stack_end__,
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
  svc,				/* SVCall */
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
  chx_handle_intr,		/* USB LP */
  /* ... and more.  CAN, EXT9_5, TIMx, I2C, SPI, USART, EXT15_10 */
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,  chx_handle_intr,
  chx_handle_intr,  chx_handle_intr,
};
