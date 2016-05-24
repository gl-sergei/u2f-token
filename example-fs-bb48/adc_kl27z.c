/*
 * adc_kl27z.c - ADC driver for KL27Z
 *               In this ADC driver, there are NeuG specific parts.
 *               You need to modify to use this as generic ADC driver.
 *
 * Copyright (C) 2016  Flying Stone Technology
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
#include "kl_sim.h"

#define INTR_REQ_ADC 15

struct ADC {
  volatile uint32_t SC1[2];/* Status and Control Registers 1 */
  volatile uint32_t CFG1;  /* Configuration Register 1       */
  volatile uint32_t CFG2;  /* Configuration Register 2       */
  volatile uint32_t R[2];  /* Data Result Register           */

  /* Compare Value Registers 1, 2 */
  volatile uint32_t CV1;
  volatile uint32_t CV2;

  volatile uint32_t SC2;   /* Status and Control Register 2  */
  volatile uint32_t SC3;   /* Status and Control Register 3  */

  volatile uint32_t OFS;   /* Offset Correction Register     */
  volatile uint32_t PG;    /* Plus-Side Gain Register        */
  volatile uint32_t MG;    /* Minus-Side Gain Register       */

  /* Plus-Side General Calibration Value Registers */
  volatile uint32_t CLPD;
  volatile uint32_t CLPS;
  volatile uint32_t CLP4;
  volatile uint32_t CLP3;
  volatile uint32_t CLP2;
  volatile uint32_t CLP1;
  volatile uint32_t CLP0;
  uint32_t rsvd0;
  /* Minus-Side General Calibration Value Registers */
  volatile uint32_t CLMD;
  volatile uint32_t CLMS;
  volatile uint32_t CLM4;
  volatile uint32_t CLM3;
  volatile uint32_t CLM2;
  volatile uint32_t CLM1;
  volatile uint32_t CLM0;
};
static struct ADC *const ADC = (struct ADC *const)0x4003B000;

/* SC1 */
#define ADC_SC1_DIFF            (1 << 5)
#define ADC_SC1_AIEN            (1 << 6)
#define ADC_SC1_COCO            (1 << 7)
#define ADC_SC1_TEMPSENSOR      26
#define ADC_SC1_BANDGAP         27
#define ADC_SC1_ADCSTOP         31

/* CFG1 */
#define ADC_CLOCK_SOURCE_ASYNCH (3 << 0)
#define ADC_MODE_16BIT          (3 << 2)
#define ADC_ADLSMP_SHORT        (0 << 4)
#define ADC_ADLSMP_LONG         (1 << 4)
#define ADC_ADIV_1              (0 << 5)
#define ADC_ADIV_8              (3 << 5)
#define ADC_ADLPC_NORMAL        (0 << 7)
#define ADC_ADLPC_LOWPOWER      (1 << 7)
/**/
#define ADC_CLOCK_SOURCE ADC_CLOCK_SOURCE_ASYNCH
#define ADC_MODE         ADC_MODE_16BIT
#if 0
#define ADC_ADLSMP       ADC_ADLSMP_SHORT
#else
#define ADC_ADLSMP       ADC_ADLSMP_LONG
#endif
#define ADC_ADIV         ADC_ADIV_8
#define ADC_ADLPC        ADC_ADLPC_LOWPOWER

/* CFG2 */
#define ADC_ADLSTS_DEFAULT      0 /* 24 cycles if CFG1.ADLSMP=1, 4 if not.  */
#define ADC_ADHSC_NORMAL        (0 << 2)
#define ADC_ADHSC_HIGHSPEED     (1 << 2)
#define ADC_ADACK_DISABLE       (0 << 3)
#define ADC_ADACK_ENABLE        (1 << 3)
#define ADC_MUXSEL_A            (0 << 4)
#define ADC_MUXSEL_B            (1 << 4)
/**/
#define ADC_ADLSTS       ADC_ADLSTS_DEFAULT 
#define ADC_ADHSC        ADC_ADHSC_NORMAL
#define ADC_ADACKEN      ADC_ADACK_ENABLE
#define ADC_MUXSEL       ADC_MUXSEL_A

/* SC2 */
#define ADC_SC2_REFSEL_DEFAULT  1 /* Internal Voltage Reference??? */
#define ADC_SC2_DMAEN           (1 << 2)
#define ADC_SC2_ACREN           (1 << 3)
#define ADC_SC2_ACFGT           (1 << 4)
#define ADC_SC2_ACFE            (1 << 5)
#define ADC_SC2_ADTRG		(1 << 6) /* For hardware trigger */

/* SC3 */
#define ADC_SC3_AVGS11          0x03
#define ADC_SC3_AVGE            (1 << 2)
#define ADC_SC3_ADCO            (1 << 3)
#define ADC_SC3_CALF            (1 << 6)
#define ADC_SC3_CAL             (1 << 7)

/*
 * Initialize ADC module, do calibration.
 * This is called by MAIN, only once, before creating any other threads.
 */
int
adc_init (void)
{
  uint32_t v;

  /* Enable ADC0 clock.  */
  SIM->SCGC6 |= (1 << 27);

  ADC->CFG1 = ADC_CLOCK_SOURCE | ADC_MODE | ADC_ADLSMP | ADC_ADIV | ADC_ADLPC;
  ADC->CFG2 = ADC_ADLSTS | ADC_ADHSC | ADC_ADACKEN | ADC_MUXSEL;
  ADC->SC2 = ADC_SC2_REFSEL_DEFAULT;
  ADC->SC3 = ADC_SC3_CAL | ADC_SC3_CALF | ADC_SC3_AVGE | ADC_SC3_AVGS11;

  /* Wait ADC completion */
  while ((ADC->SC1[0] & ADC_SC1_COCO) == 0)
    if ((ADC->SC3 & ADC_SC3_CALF) != 0)
      /* Calibration failure */
      return -1;

  if ((ADC->SC3 & ADC_SC3_CALF) != 0)
    /* Calibration failure */
    return -1;

  /* Configure PG by the calibration values.  */
  v = ADC->CLP0 + ADC->CLP1 + ADC->CLP2 + ADC->CLP3 + ADC->CLP4 + ADC->CLPS;
  ADC->PG = 0x8000 | (v >> 1);

  /* Configure MG by the calibration values.  */
  v = ADC->CLM0 + ADC->CLM1 + ADC->CLM2 + ADC->CLM3 + ADC->CLM4 + ADC->CLMS;
  ADC->MG = 0x8000 | (v >> 1);

  return 0;
}

/*
 * Start using ADC.
 */
void
adc_start (void)
{
  ADC->CFG1 = ADC_CLOCK_SOURCE | ADC_MODE | ADC_ADLSMP | ADC_ADIV | ADC_ADLPC;
  ADC->CFG2 = ADC_ADLSTS | ADC_ADHSC | ADC_ADACKEN | ADC_MUXSEL;
  ADC->SC2 = ADC_SC2_REFSEL_DEFAULT;
  ADC->SC3 = 0;
}

/*
 * Buffer to save ADC data.
 */
uint32_t adc_buf[64];

/*
 * Kick getting data for COUNT times.
 * Data will be saved in ADC_BUF starting at OFFSET.
 */
void
adc_start_conversion (int offset, int count)
{
  ADC->SC1[0] = /*ADC_SC1_AIEN*/0 | ADC_SC1_TEMPSENSOR;
}


static void
adc_stop_conversion (void)
{
  ADC->SC1[0] = ADC_SC1_ADCSTOP;
}

/*
 * Stop using ADC.
 */
void
adc_stop (void)
{
  SIM->SCGC6 &= ~(1 << 27);
}

/*
 * Return 0 on success.
 * Return 1 on error.
 */
int
adc_wait_completion (chopstx_intr_t *intr)
{
  /* Wait ADC completion */
  while ((ADC->SC1[0] & ADC_SC1_COCO) == 0)
    ;

  adc_buf[0] = ADC->R[0];
  adc_stop_conversion ();
  return 0;
}
