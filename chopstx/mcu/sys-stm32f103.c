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

#include "mcu/clk_gpio_init-stm32.c"


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
flash_program_halfword (uintptr_t addr, uint16_t data)
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
flash_erase_page (uintptr_t addr)
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
flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len)
{
  int status;
#if defined(STM32F103_OVERRIDE_FLASH_SIZE_KB)
  uintptr_t flash_end = FLASH_START_ADDR + STM32F103_OVERRIDE_FLASH_SIZE_KB*1024;
#else
  uintptr_t flash_end = FLASH_START_ADDR + (*FLASH_SIZE_REG)*1024;
#endif

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
  uintptr_t addr = FLASH_START;
#if defined(STM32F103_OVERRIDE_FLASH_SIZE_KB)
  uintptr_t end = FLASH_START_ADDR + STM32F103_OVERRIDE_FLASH_SIZE_KB*1024;
#else
  uintptr_t end = FLASH_START_ADDR + (*FLASH_SIZE_REG)*1024;
#endif
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
static struct SCB *const SCB = (struct SCB *)SCB_BASE;

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
  extern const uint32_t FT0[256], FT1[256], FT2[256];

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

/*
 * aes-constant-ft.c - AES forward tables.
 *
 * We need something useful for the initial flash ROM page (4 Ki
 * bytes), which cannot be modified after installation.  Even after
 * upgrade of the firmware, it stays intact.
 *
 * We decide to put 3/4 of AES forward tables to fill 3 Ki bytes, as
 * its useful and it won't change.
 *
 * The code was taken from aes.c of PolarSSL version 0.14, and then,
 * modified to add section names.
 *
 * Since this is just a data, it wouldn't be copyright-able, but the
 * original auther would claim so.  Thus, we put original copyright
 * notice here.  It is highly likely that there will be no such a
 * thing for copyright.  Nevertheless, we think that PolarSSL is good
 * software to address here, and encourage people using it.
 *
 */

#include <stdint.h>

/*
 * Original copyright notice is below:
 */

/*
 *  FIPS-197 compliant AES implementation
 *
 *  Copyright (C) 2006-2010, Brainspark B.V.
 *
 *  This file is part of PolarSSL (http://www.polarssl.org)
 *  Lead Maintainer: Paul Bakker <polarssl_maintainer at polarssl.org>
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 *  The AES block cipher was designed by Vincent Rijmen and Joan Daemen.
 *
 *  http://csrc.nist.gov/encryption/aes/rijndael/Rijndael.pdf
 *  http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf
 */

/*
 * Forward tables
 */
#define FT \
\
    V(A5,63,63,C6), V(84,7C,7C,F8), V(99,77,77,EE), V(8D,7B,7B,F6), \
    V(0D,F2,F2,FF), V(BD,6B,6B,D6), V(B1,6F,6F,DE), V(54,C5,C5,91), \
    V(50,30,30,60), V(03,01,01,02), V(A9,67,67,CE), V(7D,2B,2B,56), \
    V(19,FE,FE,E7), V(62,D7,D7,B5), V(E6,AB,AB,4D), V(9A,76,76,EC), \
    V(45,CA,CA,8F), V(9D,82,82,1F), V(40,C9,C9,89), V(87,7D,7D,FA), \
    V(15,FA,FA,EF), V(EB,59,59,B2), V(C9,47,47,8E), V(0B,F0,F0,FB), \
    V(EC,AD,AD,41), V(67,D4,D4,B3), V(FD,A2,A2,5F), V(EA,AF,AF,45), \
    V(BF,9C,9C,23), V(F7,A4,A4,53), V(96,72,72,E4), V(5B,C0,C0,9B), \
    V(C2,B7,B7,75), V(1C,FD,FD,E1), V(AE,93,93,3D), V(6A,26,26,4C), \
    V(5A,36,36,6C), V(41,3F,3F,7E), V(02,F7,F7,F5), V(4F,CC,CC,83), \
    V(5C,34,34,68), V(F4,A5,A5,51), V(34,E5,E5,D1), V(08,F1,F1,F9), \
    V(93,71,71,E2), V(73,D8,D8,AB), V(53,31,31,62), V(3F,15,15,2A), \
    V(0C,04,04,08), V(52,C7,C7,95), V(65,23,23,46), V(5E,C3,C3,9D), \
    V(28,18,18,30), V(A1,96,96,37), V(0F,05,05,0A), V(B5,9A,9A,2F), \
    V(09,07,07,0E), V(36,12,12,24), V(9B,80,80,1B), V(3D,E2,E2,DF), \
    V(26,EB,EB,CD), V(69,27,27,4E), V(CD,B2,B2,7F), V(9F,75,75,EA), \
    V(1B,09,09,12), V(9E,83,83,1D), V(74,2C,2C,58), V(2E,1A,1A,34), \
    V(2D,1B,1B,36), V(B2,6E,6E,DC), V(EE,5A,5A,B4), V(FB,A0,A0,5B), \
    V(F6,52,52,A4), V(4D,3B,3B,76), V(61,D6,D6,B7), V(CE,B3,B3,7D), \
    V(7B,29,29,52), V(3E,E3,E3,DD), V(71,2F,2F,5E), V(97,84,84,13), \
    V(F5,53,53,A6), V(68,D1,D1,B9), V(00,00,00,00), V(2C,ED,ED,C1), \
    V(60,20,20,40), V(1F,FC,FC,E3), V(C8,B1,B1,79), V(ED,5B,5B,B6), \
    V(BE,6A,6A,D4), V(46,CB,CB,8D), V(D9,BE,BE,67), V(4B,39,39,72), \
    V(DE,4A,4A,94), V(D4,4C,4C,98), V(E8,58,58,B0), V(4A,CF,CF,85), \
    V(6B,D0,D0,BB), V(2A,EF,EF,C5), V(E5,AA,AA,4F), V(16,FB,FB,ED), \
    V(C5,43,43,86), V(D7,4D,4D,9A), V(55,33,33,66), V(94,85,85,11), \
    V(CF,45,45,8A), V(10,F9,F9,E9), V(06,02,02,04), V(81,7F,7F,FE), \
    V(F0,50,50,A0), V(44,3C,3C,78), V(BA,9F,9F,25), V(E3,A8,A8,4B), \
    V(F3,51,51,A2), V(FE,A3,A3,5D), V(C0,40,40,80), V(8A,8F,8F,05), \
    V(AD,92,92,3F), V(BC,9D,9D,21), V(48,38,38,70), V(04,F5,F5,F1), \
    V(DF,BC,BC,63), V(C1,B6,B6,77), V(75,DA,DA,AF), V(63,21,21,42), \
    V(30,10,10,20), V(1A,FF,FF,E5), V(0E,F3,F3,FD), V(6D,D2,D2,BF), \
    V(4C,CD,CD,81), V(14,0C,0C,18), V(35,13,13,26), V(2F,EC,EC,C3), \
    V(E1,5F,5F,BE), V(A2,97,97,35), V(CC,44,44,88), V(39,17,17,2E), \
    V(57,C4,C4,93), V(F2,A7,A7,55), V(82,7E,7E,FC), V(47,3D,3D,7A), \
    V(AC,64,64,C8), V(E7,5D,5D,BA), V(2B,19,19,32), V(95,73,73,E6), \
    V(A0,60,60,C0), V(98,81,81,19), V(D1,4F,4F,9E), V(7F,DC,DC,A3), \
    V(66,22,22,44), V(7E,2A,2A,54), V(AB,90,90,3B), V(83,88,88,0B), \
    V(CA,46,46,8C), V(29,EE,EE,C7), V(D3,B8,B8,6B), V(3C,14,14,28), \
    V(79,DE,DE,A7), V(E2,5E,5E,BC), V(1D,0B,0B,16), V(76,DB,DB,AD), \
    V(3B,E0,E0,DB), V(56,32,32,64), V(4E,3A,3A,74), V(1E,0A,0A,14), \
    V(DB,49,49,92), V(0A,06,06,0C), V(6C,24,24,48), V(E4,5C,5C,B8), \
    V(5D,C2,C2,9F), V(6E,D3,D3,BD), V(EF,AC,AC,43), V(A6,62,62,C4), \
    V(A8,91,91,39), V(A4,95,95,31), V(37,E4,E4,D3), V(8B,79,79,F2), \
    V(32,E7,E7,D5), V(43,C8,C8,8B), V(59,37,37,6E), V(B7,6D,6D,DA), \
    V(8C,8D,8D,01), V(64,D5,D5,B1), V(D2,4E,4E,9C), V(E0,A9,A9,49), \
    V(B4,6C,6C,D8), V(FA,56,56,AC), V(07,F4,F4,F3), V(25,EA,EA,CF), \
    V(AF,65,65,CA), V(8E,7A,7A,F4), V(E9,AE,AE,47), V(18,08,08,10), \
    V(D5,BA,BA,6F), V(88,78,78,F0), V(6F,25,25,4A), V(72,2E,2E,5C), \
    V(24,1C,1C,38), V(F1,A6,A6,57), V(C7,B4,B4,73), V(51,C6,C6,97), \
    V(23,E8,E8,CB), V(7C,DD,DD,A1), V(9C,74,74,E8), V(21,1F,1F,3E), \
    V(DD,4B,4B,96), V(DC,BD,BD,61), V(86,8B,8B,0D), V(85,8A,8A,0F), \
    V(90,70,70,E0), V(42,3E,3E,7C), V(C4,B5,B5,71), V(AA,66,66,CC), \
    V(D8,48,48,90), V(05,03,03,06), V(01,F6,F6,F7), V(12,0E,0E,1C), \
    V(A3,61,61,C2), V(5F,35,35,6A), V(F9,57,57,AE), V(D0,B9,B9,69), \
    V(91,86,86,17), V(58,C1,C1,99), V(27,1D,1D,3A), V(B9,9E,9E,27), \
    V(38,E1,E1,D9), V(13,F8,F8,EB), V(B3,98,98,2B), V(33,11,11,22), \
    V(BB,69,69,D2), V(70,D9,D9,A9), V(89,8E,8E,07), V(A7,94,94,33), \
    V(B6,9B,9B,2D), V(22,1E,1E,3C), V(92,87,87,15), V(20,E9,E9,C9), \
    V(49,CE,CE,87), V(FF,55,55,AA), V(78,28,28,50), V(7A,DF,DF,A5), \
    V(8F,8C,8C,03), V(F8,A1,A1,59), V(80,89,89,09), V(17,0D,0D,1A), \
    V(DA,BF,BF,65), V(31,E6,E6,D7), V(C6,42,42,84), V(B8,68,68,D0), \
    V(C3,41,41,82), V(B0,99,99,29), V(77,2D,2D,5A), V(11,0F,0F,1E), \
    V(CB,B0,B0,7B), V(FC,54,54,A8), V(D6,BB,BB,6D), V(3A,16,16,2C)

#define V(a,b,c,d) 0x##a##b##c##d
const uint32_t FT0[256] __attribute__((section(".sys.0"))) = { FT };
#undef V

#define V(a,b,c,d) 0x##b##c##d##a
const uint32_t FT1[256] __attribute__((section(".sys.1"))) = { FT };
#undef V

#define V(a,b,c,d) 0x##c##d##a##b
const uint32_t FT2[256] __attribute__((section(".sys.2"))) = { FT };
#undef V

#ifdef ORIGINAL_IMPLEMENTATION 
#define V(a,b,c,d) 0x##d##a##b##c
const uint32_t FT3[256] = { FT };
#undef V
#endif
