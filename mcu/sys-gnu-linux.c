#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "board.h"

const uint8_t sys_version[8] = {
  3*2+2,	     /* bLength */
  0x03,		     /* bDescriptorType = USB_STRING_DESCRIPTOR_TYPE */
  /* sys version: "3.0" */
  '3', 0, '.', 0, '0', 0,
};

const uint32_t sys_board_id = BOARD_ID;
const uint8_t sys_board_name[] = BOARD_NAME;

void
set_led (int on)
{
  puts (on ? "*": "");
}

void
flash_unlock (void)
{
}

int
flash_program_halfword (uint32_t addr, uint16_t data)
{
  fprintf (stderr, "flash_program_halfword: addr=%08x, data=%04x\n", addr, data);
  return 0;
}

int
flash_erase_page (uint32_t addr)
{
  fprintf (stderr, "flash_erase_page: addr=%08x\n", addr);
  return 0;
}

int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  fprintf (stderr, "flash_check_blank: addr=%p, size=%zd\n", p_start, size);
  return 0;
}

int
flash_write (uint32_t dst_addr, const uint8_t *src, size_t len)
{
  fprintf (stderr, "flash_write: addr=%08x, %p, %zd\n", dst_addr, src, len);
  return 0;
}

int
flash_protect (void)
{
  fprintf (stderr, "flash_protect\n");
  return 0;
}

void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void))
{
  (void)entry;
  exit (1);
}

void
usb_lld_sys_init (void)
{
}

void
usb_lld_sys_shutdown (void)
{
}


void __attribute__((noreturn))
nvic_system_reset (void)
{
  exit (0);
}


void
clock_init (void)
{
}

void
gpio_init (void)
{
}
