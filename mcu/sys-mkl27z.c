/*
 * sys.c - First pages for MKL27Z256.
 *
 * Copyright (C) 2016 Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty provided the copyright
 * notice and this notice are preserved.  This file is offered as-is,
 * without any warranty.
 *
 *
 * First two pages of Flash ROM is difficult to use because of
 * predefined purposes.  It's defined as a default vector page and
 * a flash configuration page.
 *
 * We put something useful to those two pages, together with the
 * data for predefined purposes.
 */

#include <stdint.h>
#include <stdlib.h>
#include "board.h"

#define ADDR_VECTORS (0x00000900)
#define ADDR_SCR_VTOR 0xe000ed08

static void __attribute__ ((naked,section(".fixed_function.reset")))
reset (void)
{
  uint32_t r3 = ADDR_SCR_VTOR;

  asm volatile ("str	%2, [%0]\n\t"	  /* Set SCR->VTOR     */
		"ldr	%0, [%2]\n\t"     /* Stack address     */
		"msr	MSP, %0\n\t"	  /* Exception handler stack. */
		"ldr	%0, [%2, #4]\n\t" /* The entry address */
		"bx	%0\n\t"           /* Jump to the entry */
		".align	2"
		: "=r" (r3)
		: "0" (r3), "r" (ADDR_VECTORS)
		: "memory");

  /* Never reach here. */
}


static uint32_t
stack_entry[] __attribute__ ((section(".first_page.first_words"),used)) = {
  /* Since MSP are soon modified in RESET, we put 0 here.  */
  0,
  (uint32_t)reset,
};

#include "mcu/clk_gpio_init-mkl27z.c"

static void
set_led (int on)
{
  if (on)
    GPIOB->PCOR = (1 << 0); /* PTB0: Clear: Light on  */
  else
    GPIOB->PSOR = (1 << 0); /* PTB0: Set  : Light off */
}

/*
 * Here comes other SYS routines and data.
 */

const uint8_t sys_version[8] __attribute__((section(".sys.version"))) = {
  3*2+2,	     /* bLength */
  0x03,		     /* bDescriptorType = STRING_DESCRIPTOR */
  /* sys version: "3.0" */
  '3', 0, '.', 0, '0', 0,
};

static const uint8_t board_name_string[] = BOARD_NAME;

const uint8_t __attribute__((section(".sys.board_info")))
*const sys_board_name = board_name_string;

const uint32_t __attribute__((section(".sys.board_info")))
sys_board_id = BOARD_ID;

typedef void (*handler)(void);

handler sys_vector[] __attribute__ ((section(".sys.vectors"))) = {
  clock_init,
  gpio_init,
  (handler)set_led,
  NULL,
};

static uint32_t
flash_config[] __attribute__ ((section(".flash_config"),used)) = {
  0xffffffff, 0xffffffff, /* Comparison Key */
  0xffffffff, /* Protection bytes */
  0xffff3ffe, /* FSEC=0xfe, FOPT=0x3f */
  /* FOPT=0x3f:
   * BOOTSRC_SEL=00: Boot from flash
   */
  /* FSEC=0xfe:
   * unsecure
   */
};


/*
 * Flash memory routine
 */
struct FTFA {
  volatile uint8_t FSTAT;
  volatile uint8_t FCNFG;
  volatile uint8_t FSEC;
  volatile uint8_t FOPT;
  /* Note: addressing (3,2,1,0, 7,6,5,4, B,A,9,8) */
  /* Use Bx macro. */
  volatile uint8_t FCCO[12];
  /* Note: addressing (3,2,1,0).  Use Bx macro.  */
  volatile uint8_t FPROT[4];
};
static struct FTFA *const FTFA = (struct FTFA *)0x40020000;

#define FSTAT_CCIF 0x80
#define B3 0
#define B2 1
#define B1 2
#define B0 3
#define B7 4
#define B6 5
#define B5 6
#define B4 7
#define BB 8
#define BA 9
#define B9 10
#define B8 11

uint32_t __attribute__ ((naked,section(".fixed_function.flash_do_internal")))
flash_do_internal (void)
{
#ifdef ORIGINAL_IN_C
  uint8_t r;

  asm volatile ("cpsid	i" : : : "memory");
  FTFA->FSTAT |= FSTAT_CCIF;
  while (((r = FTFA->FSTAT) & FSTAT_CCIF) == 0)
    ;
  r &= ~FSTAT_CCIF;
  asm volatile ("cpsie	i" : : : "memory");

  return (uint32_t)r;
#else
  register unsigned int r0 asm ("r0");
  register unsigned int r1 asm ("r1") = (unsigned int)FTFA;
  register unsigned int r2 asm ("r2") = FSTAT_CCIF;

  asm volatile ("cpsid	i\n\t"
		"ldrb	%0, [%1]\n\t"
		"orr	%0, %2\n\t"
		"strb	%0, [%1]\n"
	"0:\t"
		"ldrb	%0, [%1]\n\t"
		"uxtb	%0, %0\n\t"
		"tst	%0, %2\n\t"
		"beq	0b\n\t"
		"cpsie	i\n\t"
		"bic	%0, %2\n\t"
		"bx	lr"
		: "=r" (r0)
		: "r" (r1), "r" (r2)
		: "memory");
  return r0;
#endif
}

/*
 * Let execute flash command.
 * Since the code should be on RAM, we copy the code onto stack
 * and let it go.
 */
uint32_t __attribute__ ((naked,section(".fixed_function.flash_do")))
flash_do (void)
{
  register unsigned int r0 asm ("r0");
  register unsigned int r1 asm ("r1");
  register unsigned int r2 asm ("r2");
  register unsigned int r3 asm ("r3") = (unsigned int)flash_do_internal&~3;
  /* Address of Thumb code &1 == 1, so, we clear the last bits.    ------^ */

  asm volatile ("sub	sp, #32\n\t"
		"mov	%1, sp\n\t"
		"mov	%2, #0\n"
	"0:\t"
		"cmp	%2, #32\n\t"
		"beq	1f\n\t"
		"ldr	%0, [%3, %2]\n\t"
		"str	%0, [%1, %2]\n\t"
		"add	%2, #4\n\t"
		"b	0b\n"
	"1:\t"
		"add	%1, #1\n\t" /* Thumb code requires LSB=1.  */
		"mov	%3, lr\n\t"
		"blx	%1\n\t"
		"add	sp, #32\n\t"
		"bx	%3"
		: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
		: "3" (r3));
}

#define FLASH_COMMAND_PROGRAM_LONGWORD   0x06
#define FLASH_COMMAND_ERASE_FLASH_SECTOR 0x09

int __attribute__ ((naked,section(".fixed_function.flash_erase_page")))
flash_erase_page (uint32_t addr)
{
#ifdef ORIGINAL_IN_C
  FTFA->FCCO[B0] = FLASH_COMMAND_ERASE_FLASH_SECTOR;
  FTFA->FCCO[B3] = (addr >>  0) & 0xff;
  FTFA->FCCO[B2] = (addr >>  8) & 0xff;
  FTFA->FCCO[B1] = (addr >> 16) & 0xff;
  flash_do ();
#else
  register unsigned int r0 asm ("r0") = addr;
  register unsigned int r1 asm ("r1");
  register unsigned int r2 asm ("r2") = FLASH_COMMAND_ERASE_FLASH_SECTOR;
  register unsigned int r3 asm ("r3") = (unsigned int)FTFA;

  asm volatile ("strb	%2, [%3, #7]\n\t"
		"strb	%0, [%3, #4]\n\t"
		"lsr	%0, #8\n\t"
		"strb	%0, [%3, #5]\n\t"
		"lsr	%0, #8\n\t"
		"strb	%0, [%3, #6]\n\t"
		"b	flash_do"
		: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
		: "0" (r0), "2" (r2), "3" (r3));
#endif
}

int __attribute__ ((naked,section(".fixed_function.flash_program_word")))
flash_program_word (uint32_t addr, uint32_t word)
{
#ifdef ORIGINAL_IN_C
  FTFA->FCCO[B0] = FLASH_COMMAND_PROGRAM_LONGWORD;
  FTFA->FCCO[B3] = (addr >>  0) & 0xff;
  FTFA->FCCO[B2] = (addr >>  8) & 0xff;
  FTFA->FCCO[B1] = (addr >> 16) & 0xff;
  FTFA->FCCO[B4] = (word >>  0) & 0xff;
  FTFA->FCCO[B5] = (word >>  8) & 0xff;
  FTFA->FCCO[B6] = (word >> 16) & 0xff;
  FTFA->FCCO[B7] = (word >> 24) & 0xff;
  flash_do ();
#else
  register unsigned int r0 asm ("r0") = addr;
  register unsigned int r1 asm ("r1") = word;
  register unsigned int r2 asm ("r2") = FLASH_COMMAND_PROGRAM_LONGWORD;
  register unsigned int r3 asm ("r3") = (unsigned int)FTFA;

  asm volatile ("strb	%2, [%3, #7]\n\t"
		"strb	%0, [%3, #4]\n\t"
		"lsr	%0, #8\n\t"
		"strb	%0, [%3, #5]\n\t"
		"lsr	%0, #8\n\t"
		"strb	%0, [%3, #6]\n\t"
		"strb	%1, [%3, #11]\n\t"
		"lsr	%1, #8\n\t"
		"strb	%1, [%3, #10]\n\t"
		"lsr	%1, #8\n\t"
		"strb	%1, [%3, #9]\n\t"
		"lsr	%1, #8\n\t"
		"strb	%1, [%3, #8]\n\t"
		"b	flash_do"
		: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
		: "0" (r0), "1" (r1), "2" (r2), "3" (r3));
#endif
}


/*
 * CRC32 calculation routines.
 */
void __attribute__ ((naked,section(".fixed_function.crc32_init")))
crc32_init  (unsigned int *p)
{
#ifdef ORIGINAL_IN_C
  *p = 0xffffffff;
#else
  register unsigned int r3 asm ("r3");

  asm volatile ("mov	%0, #1\n\t"
		"neg	%0, %0\n\t"
		"str	%0, [%1]\n\t"
		"bx	lr"
		: "=r" (r3)
		: "r" (p)
		: "memory");
#endif
}

#ifdef ORIGINAL_IN_C
const unsigned int *const crc32_table= (const unsigned int *)0x00000500;
#endif

void __attribute__ ((naked,section(".fixed_function.crc32_u8")))
crc32_u8 (unsigned int *p, unsigned char v)
{
#ifdef ORIGINAL_IN_C
  *p = crc32_table[(*p & 0xff) ^ v] ^ (*p >> 8);
#else
  register unsigned int r2 asm ("r2");
  register unsigned int r3 asm ("r3");

  asm volatile ("ldrb	%2, [%4]\n\t"
		"eor	%0, %2\n\t"
		"mov	%2, #0xa0\n\t" /* (0x0500 >> 3) */
		"lsl	%0, %0, #2\n\t"
		"lsl	%2, %2, #3\n\t"
		"add	%0, %0, %2\n\t"
		"ldr	%2, [%4]\n\t"
		"ldr	%1, [%0]\n\t"
		"lsr	%2, %2, #8\n\t"
		"eor	%2, %1\n\t"
		"str	%2, [%4]\n\t"
		"bx	lr"
		: "=r" (v), "=r" (r2), "=r" (r3)
		: "0" (v), "r" (p)
		: "memory");
#endif
}

void __attribute__ ((naked,section(".fixed_function.crc32_u32")))
crc32_u32 (unsigned int *p, unsigned int u)
{
#ifdef ORIGINAL_IN_C
  crc32_u8 (p, u & 0xff);
  crc32_u8 (p, (u >> 8)& 0xff);
  crc32_u8 (p, (u >> 16)& 0xff);
  crc32_u8 (p, (u >> 24)& 0xff);
#else
  register unsigned int r3 asm ("r3");
  register unsigned int r4 asm ("r4");
  register unsigned int r5 asm ("r5");

  asm volatile ("push	{%1, %2, %3, lr}\n\t"
		"mov	%2, %0\n\t"
		"mov	%3, %5\n\t"
		"uxtb	%0, %0\n\t"
		"bl	crc32_u8\n\t"
		"lsr	%0, %2, #8\n\t"
		"mov	%5, %3\n\t"
		"uxtb	%0, %0\n\t"
		"bl	crc32_u8\n\t"
		"lsr	%0, %2, #16\n\t"
		"mov	%5, %3\n\t"
		"uxtb	%0, %0\n\t"
		"bl	crc32_u8\n\t"
		"mov	%5, %3\n\t"
		"lsr	%0, %2, #24\n\t"
		"bl	crc32_u8\n\t"
		"pop	{%1, %2, %3, pc}"
		: "=r" (u), "=r" (r3), "=r" (r4), "=r" (r5)
		: "0" (u), "r" (p)
		: "memory");
#endif
}

/*
 * Table of CRC32, generated by gen_crc_table.py
 */
const unsigned int
crc32_table[256] __attribute__ ((section(".crc32_table"))) = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2, 
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c, 
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59, 
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106, 
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433, 
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950, 
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f, 
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84, 
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e, 
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b, 
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242, 
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777, 
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9, 
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};
