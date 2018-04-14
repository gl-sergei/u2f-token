/*
 * platform.c - platform specific hacks
 *
 * Copyright (C) 2018 Sean Cross,
 *                    Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware
 * Bootloader-spcific parts were copied ftom toboot.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
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
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "sys.h"

#if defined(TARGET_TOMU)
#include "toboot.h"
#include <mcu/efm32hg.h>

static struct toboot_configuration
__attribute__ ((used, section(".toboot.config")))
toboot_config = {
  .align = 0,
  .magic = TOBOOT_V2_MAGIC,
  .reserved_gen = 0,
  .start = 16,
  .config = TOBOOT_CONFIG_FLAG_AUTORUN,
  .lock_entry = 0,
  .erase_mask_lo = 0x00000000,
  .erase_mask_hi = 0xc0000000,
  .reserved_hash = 0
};

#define LOCKBITS_BASE   (0x0FE04000UL) /* Lock-bits page base address */
#define DEBUG_LOCK_WORD (LOCKBITS_BASE + (127 * 4))

const volatile uint32_t *dlw = (volatile uint32_t *) DEBUG_LOCK_WORD;

#if defined(ENFORCE_DEBUG_LOCK)
/* Debug lock EFM32HG device by clearing DEBUG LOCK WORD */
static void
debug_lock_maybe (void)
{
  uint8_t zero[] = { 0x0, 0x0, 0x0, 0x0 };
  if (*dlw != 0)
    flash_write ((uintptr_t) dlw, zero, sizeof (zero));
}
#endif /* ENFORCE_DEBUG_LOCK */

#else /* STM32F1x obiously */

#define OPTION_BYTES_ADDR 0x1ffff800

#if defined(ENFORCE_DEBUG_LOCK)
/* Debug lock STM32F1x device */
static void
debug_lock_maybe (void)
{
  uint32_t ob_val = * (uint32_t *) (OPTION_BYTES_ADDR);

  if ((ob_val & 0xff) != 0xff)
    {
      flash_unlock ();
      flash_protect ();
      nvic_system_reset ();
    }
}
#endif /* ENFORCE_DEBUG_LOCK */

#endif /* TARGET_TOMU */

/* Preform platform-specific actions */
void
platform_init (void)
{

#if defined(ENFORCE_DEBUG_LOCK)
  debug_lock_maybe ();
#endif
}
