/*
 * adc-efm32.c - ADC driver for EFM32HG
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
#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <mcu/efm32.h>

uint32_t adc_buf[64];

static chopstx_intr_t adc_intr;

struct adc_req
{
  int offset;
  int count;
};

static struct adc_req req;

#define INTR_REQ_ADC0       4

int adc_init (void)
{
  /* Enable ADC0 clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC0;

  /* Make sure conversion is not in progress */
  ADC0->CMD = ADC_CMD_SINGLESTOP | ADC_CMD_SCANSTOP;

  const uint8_t timebase = MHZ + 1; /* we are using HFRCO as HFCLK source */
  const uint8_t presc = 2;

  ADC0->CTRL = (0 << 24)           /* OVSRSEL */
               | (timebase << 16)  /* TIMEBASE */
               | (presc << 8)      /* PRESC */
               | (0 << 0)          /* WARMUPMODE */;

  ADC0->SINGLECTRL = (1 << 28)    /* PRS channel 1 (we use ch0 for capsense) */
                   | (0 << 20)    /* 1 cycle aacquisition time */
                   | (0 << 16)    /* 1.25V internal reference */
                   | (0 <<  4)    /* 12 bit resolution */
                   | (8 <<  8)    /* temperature sensor input selected */
                   | (0 <<  1)    /* differential mode */
                   | (0 << 24)    /* disable PRS */
                   | (0 <<  2)    /* right adjust */
                   | (1 <<  0);   /* repetitive mode */

  ADC0->CAL = DEVINFO->ADC0CAL0;

  return 0;
}

void adc_start (void)
{
  /* do nothing */
}

void adc_stop (void)
{
  /* do nothing */
}

void adc_start_conversion (int offset, int count)
{
  req.offset = offset;
  req.count = count;
}

extern uint8_t u;
int adc_wait_completion (void)
{
  struct chx_poll_head *pd_array[1] = { (struct chx_poll_head *)&adc_intr };
  uint32_t value;
  int samples;

  chopstx_claim_irq (&adc_intr, INTR_REQ_ADC0);

  ADC0->IFC = 0xffffffff;
  ADC0->IEN = 1; /* enable single conversion complete interrupt */
  ADC0->CMD = ADC_CMD_SINGLESTART;

  value = 0;
  samples = 0;
  while (req.count > 0)
    {
      /* Wait for single conversion completion */
      chopstx_poll (NULL, 1, pd_array);

      if (adc_intr.ready && (ADC0->IF & 1))
        {
          /* Combine 4 12-bit samples into single 32-bit one. Entropy 
          comes from less significant bits. */
          if (samples < 4)
            {
              value = (value << 8) | (ADC0->SINGLEDATA & 0xff);
              ++samples;
            }
          else
            {
              adc_buf[req.offset++] = value;
              --req.count;
              value = 0;
              samples = 0;
            }
        }
    }

  ADC0->IEN = 0;
  ADC0->CMD = ADC_CMD_SINGLESTOP;

  return 0;
}
