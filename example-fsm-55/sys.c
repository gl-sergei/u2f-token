/*
 * sys.c - system routines for the initial page for STM32F030 / STM32F103.
 *
 * Copyright (C) 2013, 2014 Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty provided the copyright
 * notice and this notice are preserved.  This file is offered as-is,
 * without any warranty.
 *
 * When the flash ROM is protected, we cannot modify the initial page.
 * We put some system routines (which is useful for any program) here.
 */

#include <stdint.h>
#include <stdlib.h>
#include "board.h"


#define CORTEX_PRIORITY_BITS    4
#define CORTEX_PRIORITY_MASK(n)  ((n) << (8 - CORTEX_PRIORITY_BITS))
#define USB_LP_CAN1_RX0_IRQn	 20
#define STM32_USB_IRQ_PRIORITY   11

#define STM32_SW_HSI		(0 << 0)
#define STM32_SW_PLL		(2 << 0)
#define STM32_PLLSRC_HSI	(0 << 16)
#define STM32_PLLSRC_HSE	(1 << 16)

#define STM32_PLLXTPRE_DIV1	(0 << 17)
#define STM32_PLLXTPRE_DIV2	(1 << 17)

#define STM32_HPRE_DIV1		(0 << 4)

#define STM32_PPRE1_DIV1	(0 << 8)
#define STM32_PPRE1_DIV2	(4 << 8)

#define STM32_PPRE2_DIV1        (0 << 11)
#define STM32_PPRE2_DIV2	(4 << 11)

#define STM32_ADCPRE_DIV4	(1 << 14)
#define STM32_ADCPRE_DIV6       (2 << 14)

#define STM32_USBPRE_DIV1P5     (0 << 22)

#define STM32_MCO_NOCLOCK	(0 << 24)

#if MCU_STM32F0
#define STM32_PPRE1		STM32_PPRE1_DIV1
#define STM32_PLLSRC		STM32_PLLSRC_HSI
#define STM32_FLASHBITS		0x00000011
#define STM32_PLLCLKIN		(STM32_HSICLK / 2)
#else
#define STM32_PPRE1		STM32_PPRE1_DIV2
#define STM32_PLLSRC		STM32_PLLSRC_HSE
#define STM32_FLASHBITS		0x00000012
#define STM32_PLLCLKIN		(STM32_HSECLK / 1)
#endif

#define STM32_SW		STM32_SW_PLL
#define STM32_HPRE		STM32_HPRE_DIV1
#define STM32_PPRE2		STM32_PPRE2_DIV1
#define STM32_ADCPRE		STM32_ADCPRE_DIV6
#define STM32_MCOSEL		STM32_MCO_NOCLOCK
#define STM32_USBPRE            STM32_USBPRE_DIV1P5

#define STM32_PLLMUL		((STM32_PLLMUL_VALUE - 2) << 18)
#define STM32_PLLCLKOUT		(STM32_PLLCLKIN * STM32_PLLMUL_VALUE)
#define STM32_SYSCLK		STM32_PLLCLKOUT
#define STM32_HCLK		(STM32_SYSCLK / 1)

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

static struct NVIC *const NVICBase = ((struct NVIC *const)0xE000E100);
#define NVIC_ISER(n)	(NVICBase->ISER[n >> 5])
#define NVIC_ICPR(n)	(NVICBase->ICPR[n >> 5])
#define NVIC_IPR(n)	(NVICBase->IPR[n >> 2])

static void
nvic_enable_vector (uint32_t n, uint32_t prio)
{
  unsigned int sh = (n & 3) << 3;

  NVIC_IPR (n) = (NVIC_IPR(n) & ~(0xFF << sh)) | (prio << sh);
  NVIC_ICPR (n) = 1 << (n & 0x1F);
  NVIC_ISER (n) = 1 << (n & 0x1F);
}


#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x08000000)

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
#if MCU_STM32F0
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR2;
  volatile uint32_t CFGR3;
  volatile uint32_t CR2;
#endif
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

#if MCU_STM32F0
#define RCC_AHBRSTR_IOPARST	0x00020000
#define RCC_AHBRSTR_IOPBRST	0x00040000
#define RCC_AHBRSTR_IOPCRST	0x00080000
#define RCC_AHBRSTR_IOPDRST	0x00100000
#define RCC_AHBRSTR_IOPFRST	0x00400000

#define RCC_AHBENR_IOPAEN	0x00020000
#define RCC_AHBENR_IOPBEN	0x00040000
#define RCC_AHBENR_IOPCEN	0x00080000
#define RCC_AHBENR_IOPDEN	0x00100000
#define RCC_AHBENR_IOPFEN	0x00400000

#define RCC_APB2RSTR_SYSCFGRST	0x00000001
#define RCC_APB2ENR_SYSCFGEN	0x00000001
#else
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
#endif

#if MCU_STM32F0
struct SYSCFG {
  volatile uint32_t CFGR1;
  uint32_t dummy0;
  volatile uint32_t EXTICR[4];
  volatile uint32_t CFGR2;
};
#define SYSCFG_CFGR1_MEM_MODE 0x03

#define SYSCFG_BASE	(APBPERIPH_BASE + 0x00010000)
static struct SYSCFG *const SYSCFG = ((struct SYSCFG *const) SYSCFG_BASE);
#endif

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

static void
clock_init (void)
{
  /* HSI setup */
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;
  /* Reset HSEON, HSEBYP, CSSON, and PLLON, not touching RCC_CR_HSITRIM */
  RCC->CR &= (RCC_CR_HSITRIM | RCC_CR_HSION);
  RCC->CFGR = 0;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;

#if !MCU_STM32F0
  /* HSE setup */
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;
#endif

  /* PLL setup */
  RCC->CFGR |= STM32_PLLMUL | STM32_PLLXTPRE | STM32_PLLSRC;
  RCC->CR   |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  /* Clock settings */
  RCC->CFGR = STM32_MCOSEL | STM32_USBPRE | STM32_PLLMUL | STM32_PLLXTPRE
    | STM32_PLLSRC | STM32_ADCPRE | STM32_PPRE2 | STM32_PPRE1 | STM32_HPRE;

  /* Switching on the configured clock source. */
  RCC->CFGR |= STM32_SW;
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;

  /*
   * We don't touch RCC->CR2, RCC->CFGR2, RCC->CFGR3, and RCC->CIR.
   */

  /* Flash setup */
  FLASH->ACR = STM32_FLASHBITS;

  /* CRC */
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

#if MCU_STM32F0
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB2RSTR = RCC_APB2RSTR_SYSCFGRST;
  RCC->APB2RSTR = 0;
  
  /* Use vectors on RAM */
  SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | 3;
#endif
}


#if MCU_STM32F0
struct GPIO {
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  uint16_t dummy0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  uint16_t dummy1;
  volatile uint16_t ODR;
  uint16_t dummy2;
  volatile uint16_t BSRR;
  uint16_t dummy3;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint16_t BRR;
  uint16_t dummy4;
};

#define GPIOA_BASE	(AHB2PERIPH_BASE + 0x0000)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE	(AHB2PERIPH_BASE + 0x0400)
#define GPIOB		((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE	(AHB2PERIPH_BASE + 0x0800)
#define GPIOC		((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE	(AHB2PERIPH_BASE + 0x0C00)
#define GPIOD		((struct GPIO *) GPIOD_BASE)
#define GPIOF_BASE	(AHB2PERIPH_BASE + 0x1400)
#define GPIOF		((struct GPIO *) GPIOF_BASE)
#else
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
#endif

static struct GPIO *const GPIO_LED = ((struct GPIO *const) GPIO_LED_BASE);
#ifdef GPIO_USB_BASE
static struct GPIO *const GPIO_USB = ((struct GPIO *const) GPIO_USB_BASE);
#endif
#ifdef GPIO_OTHER_BASE
static struct GPIO *const GPIO_OTHER = ((struct GPIO *const) GPIO_OTHER_BASE);
#endif

static void
gpio_init (void)
{
  /* Enable GPIO clock. */
#if MCU_STM32F0
  RCC->AHBENR |= RCC_ENR_IOP_EN;
  RCC->AHBRSTR = RCC_RSTR_IOP_RST;
  RCC->AHBRSTR = 0;
#else
  RCC->APB2ENR |= RCC_ENR_IOP_EN;
  RCC->APB2RSTR = RCC_RSTR_IOP_RST;
  RCC->APB2RSTR = 0;
#endif

#if MCU_STM32F0
  GPIO_LED->OSPEEDR = VAL_GPIO_OSPEEDR;
  GPIO_LED->OTYPER  = VAL_GPIO_OTYPER;
  GPIO_LED->MODER   = VAL_GPIO_MODER;
  GPIO_LED->PUPDR   = VAL_GPIO_PUPDR;

#ifdef GPIO_OTHER_BASE
  GPIO_OTHER->OSPEEDR = VAL_GPIO_OTHER_OSPEEDR;
  GPIO_OTHER->OTYPER  = VAL_GPIO_OTHER_OTYPER;
  GPIO_OTHER->MODER   = VAL_GPIO_OTHER_MODER;
  GPIO_OTHER->PUPDR   = VAL_GPIO_OTHER_PUPDR;
#endif
#else
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
#endif
}

static void
usb_cable_config (int enable)
{
#if defined(GPIO_USB_SET_TO_ENABLE)
  if (enable)
    GPIO_USB->BSRR = (1 << GPIO_USB_SET_TO_ENABLE);
  else
    GPIO_USB->BRR = (1 << GPIO_USB_SET_TO_ENABLE);
#elif defined(GPIO_USB_CLEAR_TO_ENABLE)
  if (enable)
    GPIO_USB->BRR = (1 << GPIO_USB_CLEAR_TO_ENABLE);
  else
    GPIO_USB->BSRR = (1 << GPIO_USB_CLEAR_TO_ENABLE);
#else
  (void)enable;
#endif
}

void
set_led (int on)
{
#if defined(GPIO_LED_CLEAR_TO_EMIT)
  if (on)
    GPIO_LED->BRR = (1 << GPIO_LED_CLEAR_TO_EMIT);
  else
    GPIO_LED->BSRR = (1 << GPIO_LED_CLEAR_TO_EMIT);
#else
  if (on)
    GPIO_LED->BSRR = (1 << GPIO_LED_SET_TO_EMIT);
  else
    GPIO_LED->BRR = (1 << GPIO_LED_SET_TO_EMIT);
#endif
}

static void wait (int count)
{
  int i;

  for (i = 0; i < count; i++)
    asm volatile ("" : : "r" (i) : "memory");
}


static void
usb_lld_sys_shutdown (void)
{
  RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
  RCC->APB1RSTR = RCC_APB1RSTR_USBRST;
  usb_cable_config (0);
}

static void
usb_lld_sys_init (void)
{
  if ((RCC->APB1ENR & RCC_APB1ENR_USBEN)
      && (RCC->APB1RSTR & RCC_APB1RSTR_USBRST) == 0)
    /* Make sure the device is disconnected, even after core reset.  */
    {
      usb_lld_sys_shutdown ();
      /* Disconnect requires SE0 (>= 2.5uS).  */
      wait (300);
    }

  usb_cable_config (1);
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;
  nvic_enable_vector (USB_LP_CAN1_RX0_IRQn,
		      CORTEX_PRIORITY_MASK (STM32_USB_IRQ_PRIORITY));
  /*
   * Note that we also have other IRQ(s):
   * 	USB_HP_CAN1_TX_IRQn (for double-buffered or isochronous)
   * 	USBWakeUp_IRQn (suspend/resume)
   */
  RCC->APB1RSTR = RCC_APB1RSTR_USBRST;
  RCC->APB1RSTR = 0;
}

#define FLASH_KEY1               0x45670123UL
#define FLASH_KEY2               0xCDEF89ABUL

enum flash_status
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
};

static void __attribute__ ((used))
flash_unlock (void)
{
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}


#define intr_disable()  asm volatile ("cpsid   i" : : : "memory")
#define intr_enable()  asm volatile ("cpsie   i" : : : "memory")

#define FLASH_SR_BSY		0x01
#define FLASH_SR_PGERR		0x04
#define FLASH_SR_WRPRTERR	0x10
#define FLASH_SR_EOP		0x20

#define FLASH_CR_PG	0x0001
#define FLASH_CR_PER	0x0002
#define FLASH_CR_MER	0x0004
#define FLASH_CR_OPTPG	0x0010
#define FLASH_CR_OPTER	0x0020
#define FLASH_CR_STRT	0x0040
#define FLASH_CR_LOCK	0x0080
#define FLASH_CR_OPTWRE	0x0200
#define FLASH_CR_ERRIE	0x0400
#define FLASH_CR_EOPIE	0x1000

static int
flash_wait_for_last_operation (uint32_t timeout)
{
  int status;

  do
    {
      status = FLASH->SR;
      if (--timeout == 0)
	break;
    }
  while ((status & FLASH_SR_BSY) != 0);

  return status & (FLASH_SR_BSY|FLASH_SR_PGERR|FLASH_SR_WRPRTERR);
}

#define FLASH_PROGRAM_TIMEOUT 0x00010000
#define FLASH_ERASE_TIMEOUT   0x01000000

static int
flash_program_halfword (uint32_t addr, uint16_t data)
{
  int status;

  status = flash_wait_for_last_operation (FLASH_PROGRAM_TIMEOUT);

  intr_disable ();
  if (status == 0)
    {
      FLASH->CR |= FLASH_CR_PG;

      *(volatile uint16_t *)addr = data;

      status = flash_wait_for_last_operation (FLASH_PROGRAM_TIMEOUT);
      FLASH->CR &= ~FLASH_CR_PG;
    }
  intr_enable ();

  return status;
}

static int
flash_erase_page (uint32_t addr)
{
  int status;

  status = flash_wait_for_last_operation (FLASH_ERASE_TIMEOUT);

  intr_disable ();
  if (status == 0)
    {
      FLASH->CR |= FLASH_CR_PER;
      FLASH->AR = addr;
      FLASH->CR |= FLASH_CR_STRT;

      status = flash_wait_for_last_operation (FLASH_ERASE_TIMEOUT);
      FLASH->CR &= ~FLASH_CR_PER;
    }
  intr_enable ();

  return status;
}

static int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  const uint8_t *p;

  for (p = p_start; p < p_start + size; p++)
    if (*p != 0xff)
      return 0;

  return 1;
}

extern uint8_t __flash_start__, __flash_end__;

static int
flash_write (uint32_t dst_addr, const uint8_t *src, size_t len)
{
  int status;
  uint32_t flash_start = (uint32_t)&__flash_start__;
  uint32_t flash_end = (uint32_t)&__flash_end__;

  if (dst_addr < flash_start || dst_addr + len > flash_end)
    return 0;

  while (len)
    {
      uint16_t hw = *src++;

      hw |= (*src++ << 8);
      status = flash_program_halfword (dst_addr, hw);
      if (status != 0)
	return 0;		/* error return */

      dst_addr += 2;
      len -= 2;
    }

  return 1;
}

#define OPTION_BYTES_ADDR 0x1ffff800

static int
flash_protect (void)
{
  int status;
  uint32_t option_bytes_value;

  status = flash_wait_for_last_operation (FLASH_ERASE_TIMEOUT);

  intr_disable ();
  if (status == 0)
    {
      FLASH->OPTKEYR = FLASH_KEY1;
      FLASH->OPTKEYR = FLASH_KEY2;

      FLASH->CR |= FLASH_CR_OPTER;
      FLASH->CR |= FLASH_CR_STRT;

      status = flash_wait_for_last_operation (FLASH_ERASE_TIMEOUT);
      FLASH->CR &= ~FLASH_CR_OPTER;
    }
  intr_enable ();

  if (status != 0)
    return 0;

  option_bytes_value = *(uint32_t *)OPTION_BYTES_ADDR;
  return (option_bytes_value & 0xff) == 0xff ? 1 : 0;
}

static void __attribute__((naked))
flash_erase_all_and_exec (void (*entry)(void))
{
  uint32_t addr = (uint32_t)&__flash_start__;
  uint32_t end = (uint32_t)&__flash_end__;
  int r;

  while (addr < end)
    {
      r = flash_erase_page (addr);
      if (r != 0)
	break;

      addr += FLASH_PAGE_SIZE;
    }

  if (addr >= end)
    (*entry) ();

  for (;;);
}

struct SCB
{
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile uint32_t PFR[2];
  volatile uint32_t DFR;
  volatile uint32_t ADR;
  volatile uint32_t MMFR[4];
  volatile uint32_t ISAR[5];
};

#define SCS_BASE	(0xE000E000)
#define SCB_BASE	(SCS_BASE +  0x0D00)
static struct SCB *const SCB = ((struct SCB *const) SCB_BASE);

#define SYSRESETREQ 0x04
static void
nvic_system_reset (void)
{
  SCB->AIRCR = (0x05FA0000 | (SCB->AIRCR & 0x70) | SYSRESETREQ);
  asm volatile ("dsb");
  for (;;);
}

static void __attribute__ ((naked))
reset (void)
{
  /*
   * This code may not be at start of flash ROM, because of DFU.
   * So, we take the address from PC.
   */
#if __ARM_ARCH_6M__
  asm volatile ("cpsid	i\n\t"		/* Mask all interrupts. */
		"ldr	r0, 1f\n\t"     /* r0 = RAM start */
		"mov	r1, pc\n\t"	/* r1 = (PC + 0x0400) & ~0x03ff */
		"mov	r2, #0x04\n\t"
		"lsl	r2, #8\n\t"
		"add	r1, r1, r2\n\t"
		"sub	r2, r2, #1\n\t"
		"bic	r1, r1, r2\n\t"
		"mov	r2, #188\n"
	"2:\n\t" /* Copy vectors.  It will be enabled later by clock_init.  */
		"ldr	r3, [r1, r2]\n\t"
		"str	r3, [r0, r2]\n\t"
		"sub	r2, #4\n\t"
		"bcs	2b\n\t"
		"msr	MSP, r3\n\t"	/* Main (exception handler) stack. */
		"ldr	r0, [r1, #4]\n\t" /* Reset handler.                */
		"bx	r0\n\t"
		".align	2\n"
	"1:	.word	0x20000000"
		: /* no output */ : /* no input */ : "memory");
#else
  extern const unsigned long *FT0, *FT1, *FT2;
  asm volatile ("cpsid	i\n\t"		/* Mask all interrupts. */
		"ldr	r0, 1f\n\t"     /* r0 = SCR */
		"mov	r1, pc\n\t"	/* r1 = (PC + 0x1000) & ~0x0fff */
		"mov	r2, #0x1000\n\t"
		"add	r1, r1, r2\n\t"
		"sub	r2, r2, #1\n\t"
		"bic	r1, r1, r2\n\t"
		"str	r1, [r0, #8]\n\t"	/* Set SCR->VCR */
		"ldr	r0, [r1], #4\n\t"
		"msr	MSP, r0\n\t"	/* Main (exception handler) stack. */
		"ldr	r0, [r1]\n\t"	/* Reset handler.                  */
		"bx	r0\n\t"
		".align	2\n"
	"1:	.word	0xe000ed00"
		: /* no output */ : /* no input */ : "memory");
  /* Artificial entry to refer FT0, FT1, and FT2.  */
  asm volatile (""
		: : "r" (FT0), "r" (FT1), "r" (FT2));
#endif
  /* Never reach here. */
}

typedef void (*handler)(void);
extern uint8_t __ram_end__;

handler vector[] __attribute__ ((section(".vectors"))) = {
  (handler)&__ram_end__,
  reset,
  (handler)set_led,
  flash_unlock,
  (handler)flash_program_halfword,
  (handler)flash_erase_page,
  (handler)flash_check_blank,
  (handler)flash_write,
  (handler)flash_protect,
  (handler)flash_erase_all_and_exec,
  usb_lld_sys_init,
  usb_lld_sys_shutdown,
  nvic_system_reset,
  clock_init,
  gpio_init,
  NULL,
};

const uint8_t sys_version[8] __attribute__((section(".sys.version"))) = {
  3*2+2,	     /* bLength */
  0x03,		     /* bDescriptorType = USB_STRING_DESCRIPTOR_TYPE*/
  /* sys version: "2.0" */
  '2', 0, '.', 0, '0', 0,
};
