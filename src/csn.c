/*
 * csn.c - capsense driver and user presence indicator
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware for EFM32
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
#include <chopstx.h>

#include "board.h"
#include "sys.h"

#include <mcu/efm32hg.h>

#define MASKED_WRITE(reg, mask, val) { (reg) = (((reg) & ~(mask)) | (val)); }

#define INTR_REQ_TIMER0 2
#define INTR_REQ_TIMER1 7
#define PRIO_CSN 8

extern uint8_t __process6_stack_base__[], __process6_stack_size__[];

#define STACK_ADDR_CSN ((uint32_t)__process6_stack_base__)
#define STACK_SIZE_CSN ((uint32_t)__process6_stack_size__)

static uint32_t present = 0;
static uint32_t count_max[2] = {0, 0};

static chopstx_intr_t timer1_intr;

static void
measure_start (int ch)
{
  /* select channel */
  MASKED_WRITE(ACMP0->INPUTSEL, 0x07, ch);

  /* reset and start timers */
  TIMER0->CNT = 0;
  TIMER1->CNT = 0;
  TIMER0->CMD = (1 << 0);  /* start */
  TIMER1->CMD = (1 << 0);  /* start */
}

static uint32_t
measure_stop (void)
{
  TIMER0->CMD = (1 << 1);  /* stop */
  TIMER1->CMD = (1 << 1);  /* stop */

  TIMER1->IFC = 1; /* clear interrupt flag */

  return TIMER0->CNT;
}

static void *
csn (void *arg)
{
  int ch = 0;
  int since_last_touch = 0;

  (void)arg;

  chopstx_claim_irq (&timer1_intr, INTR_REQ_TIMER1);

  measure_start (ch);

  while (1)
    {
      uint32_t touch = 0;

      struct chx_poll_head *pd_array[1] = {
        (struct chx_poll_head *)&timer1_intr
      };

      chopstx_poll (NULL, 1, pd_array);

      if (timer1_intr.ready)
        {
          uint32_t count;
          uint32_t threshold;

          count = measure_stop ();

          threshold = count_max[ch] - count_max[ch] / 2;
          if (count > 0 && count < threshold)
            touch |= 1 << ch;
          else
            touch &= ~(1 << ch);

          if (count > threshold)
            count_max[ch] = (count_max[ch] + count) / 2;

          if (present > 0)
            --present;

          if (touch)
            {
              if (since_last_touch > 10)
                {
                  if (present > 0)
                    present = 0;   /* clear user presence */
                  else
                    present = 500; /* set user presence for 10 seconds */
                }
              since_last_touch = 0;
            }
          else
            ++since_last_touch;

          if (since_last_touch > 1000)
            since_last_touch = 1000;

          ch ^= 1;

          measure_start (ch);
        }
    }

  return NULL;
}

int
user_presence_get (void)
{
  return (present > 0);
}

void
user_presence_reset (void)
{
  present = 0;
}

void
capsense_init (void)
{
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ACMP0
                      | CMU_HFPERCLKEN0_TIMER0 | CMU_HFPERCLKEN0_PRS
                      | CMU_HFPERCLKEN0_TIMER1;

  /* Set control register. No need to set interrupt modes */
  ACMP0->CTRL = (0x0 << 31)    /* FULLBIAS */
                | (0x0 << 30)  /* HALFBIAS */
                | (0x7 << 24)  /* BIASPROG */
                | (0x7 <<  8)  /* WARMTIME */
                | (0x5 <<  4); /* HYSTSEL */

  /* Select capacative sensing mode by selecting a resistor and enabling it */
  ACMP0->INPUTSEL= (0x03 << 28)   /* CSRESSEL */
                   | (0x01 << 24) /* CSRESEN */
                   | (0x00 << 16) /* LPREF */
                   | (0x3d <<  8) /* VDDLEVEL */
                   | (0x0B <<  4);/* NEGSEL = CAPSENSE */

  /* Enable ACMP if requested. */
  MASKED_WRITE(ACMP0->CTRL, 0x1, 1);

  MASKED_WRITE(ACMP0->INPUTSEL, 0x07, 0);

  while (!(ACMP0->STATUS & 0x1 /* ACMPACT */)) {};

  /* Initialize TIMER0 - Prescaler 2^10, clock source CC1, top value 0xFFFF */
  TIMER0->CTRL = (0xA << 24)               /* PRESC = DIV1024 */
                 | (1 << 16);              /* CLKSEL = CC1 */
  TIMER0->TOP  = 0xFFFF;

  /*Set up TIMER0 CC1 to trigger on PRS ch 0 */
  TIMER0->CC[1].CTRL = (1 << 0)            /* MODE = INPUTCAPTURE */
                       | (0 << 16)         /* PRSSEL = PRSCH0 */
                       | (1 << 20)         /* INSEL = PRS */
                       | (2 << 26)         /* ICEVCTRL = RISING */
                       | (2 << 24);        /* ICEDGE = BOTH */

  /* Set up PRS ch 0 to trigger on ACMP0 output */
  PRS->CH[0].CTRL = (1 << 24)              /* EDSEL = POSEDGE */
                    | (2 << 16)            /* SOURCESEL = ACMP0 */
                    | (0 << 0);            /* SIGSEL = ACMP0OUT */

  /* Initialize TIMER1 - Prescaler 2^10, top value 40 * MHZ,
  interrupt on overflow */
  TIMER1->CTRL = (0xa << 24);  /* PRESC_DIV1024 */
  TIMER1->TOP  = 40 * MHZ;
  TIMER1->IEN  = (1 << 0);     /* IEN_OF */
  TIMER1->CNT  = 0;

  chopstx_create (PRIO_CSN, STACK_ADDR_CSN, STACK_SIZE_CSN, csn, NULL);
}
