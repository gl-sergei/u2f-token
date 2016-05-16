/*
 * sys.c - system routines for the initial page for STM32F103.
 *
 * Copyright (C) 2013, 2014, 2015, 2016  Flying Stone Technology
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

#include "clk_gpio_init-stm32.c"


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

#define FLASH_START_ADDR 0x08000000 /* Fixed for all STM32F1.  */
#define FLASH_OFFSET     0x1000     /* First pages are not-writable.  */
#define FLASH_START      (FLASH_START_ADDR+FLASH_OFFSET)
#define CHIP_ID_REG      ((uint32_t *)0xe0042000)
#define FLASH_SIZE_REG   ((uint16_t *)0x1ffff7e0)

static int
flash_write (uint32_t dst_addr, const uint8_t *src, size_t len)
{
  int status;
  uint32_t flash_end = FLASH_START_ADDR + (*FLASH_SIZE_REG)*1024;

  if (dst_addr < FLASH_START || dst_addr + len > flash_end)
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
  uint32_t addr = FLASH_START;
  uint32_t end = FLASH_START_ADDR + (*FLASH_SIZE_REG)*1024;
  uint32_t page_size = 1024;
  int r;

  if (((*CHIP_ID_REG) & 0xfff) == 0x0414)
    page_size = 2048;

  while (addr < end)
    {
      r = flash_erase_page (addr);
      if (r != 0)
	break;

      addr += page_size;
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
  extern const unsigned long *FT0, *FT1, *FT2;

  /*
   * This code may not be at the start of flash ROM, because of DFU.
   * So, we take the address from PC.
   */
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

  /* Never reach here. */
  /* Artificial entry to refer FT0, FT1, and FT2.  */
  asm volatile (""
		: : "r" (FT0), "r" (FT1), "r" (FT2));
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
  0x03,		     /* bDescriptorType = USB_STRING_DESCRIPTOR_TYPE */
  /* sys version: "3.0" */
  '3', 0, '.', 0, '0', 0,
};

const uint32_t __attribute__((section(".sys.board_id")))
sys_board_id = BOARD_ID;

const uint8_t __attribute__((section(".sys.board_name")))
sys_board_name[] = BOARD_NAME;
