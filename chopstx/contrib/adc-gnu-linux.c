/*
 * adc-gnu-linux.c - ADC driver for GNU/Linux emulation.
 *                   This ADC driver just fills pseudo random values.
 *                   It's completely useless other than for NeuG.
 *
 * Copyright (C) 2017  Free Software Initiative of Japan
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
#include "adc.h"

#define ADC_RANDOM_SEED 0x01034649 /* "Hello, father!" in Japanese */

/*
 * Do calibration for ADC.
 */
int
adc_init (void)
{
  srandom (ADC_RANDOM_SEED);
  return 0;
}

void
adc_start (void)
{
}

uint32_t adc_buf[64];

void
adc_start_conversion (int offset, int count)
{
  while (count--)
    adc_buf[offset++] = random ();
}


void
adc_stop (void)
{
}


/*
 * Return 0 on success.
 * Return 1 on error.
 */
int
adc_wait_completion (void)
{
  return 0;
}
