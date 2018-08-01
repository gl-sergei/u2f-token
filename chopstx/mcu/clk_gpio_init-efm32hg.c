/*
 * clk_gpio_init-efm32.c - Clock and GPIO initialization for EFM32HG
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of Chpostx port to EFM32HG
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

#include <mcu/efm32hg.h>

static void __attribute__((used))
clock_init (void)
{
  /* disable watchdog timer (set by toboot) */
  WDOG->CTRL = 0;

  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;
  CMU->HFPERCLKDIV = 1 << 8;

#if (MHZ == 21)
  /* Set HFRCO freq 21 MHz */
  CMU->HFRCOCTRL = (4 << 8) | (DEVINFO->HFRCOCAL1 & 0xff << 0);
#elif (MHZ != 14)
  #error "Unsuppored clock frequency."
#endif

  CMU->OSCENCMD = CMU_OSCENCMD_HFRCOEN;
  while (!(CMU->STATUS & CMU_STATUS_HFRCORDY));

  CMU->CMD |= CMU_CMD_HFCLKSEL_HFRCO;
  while ((CMU->STATUS & CMU_STATUS_HFRCOSEL) == 0);
}

static void __attribute__((used))
gpio_init (void)
{
  GPIO->P[0].DOUT = 0xffffffff;
  GPIO->P[1].DOUT = 0xffffffff;
  GPIO->P[0].MODEL = 0x88888888;
  GPIO->P[0].MODEH = 0x88888888;
  GPIO->P[1].MODEL = 0x88888888;
  GPIO->P[1].MODEH = 0x88888888;
}
