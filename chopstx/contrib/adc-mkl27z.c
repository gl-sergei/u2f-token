/*
 * adc-mkl27z.c - ADC driver for MKL27Z
 *               In this ADC driver, there are NeuG specific parts.
 *               It only records lower 8-bit of 16-bit data.
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
#include <mcu/mkl27z.h>

struct DMAMUX {
  volatile uint32_t CHCFG0;
  volatile uint32_t CHCFG1;
  volatile uint32_t CHCFG2;
  volatile uint32_t CHCFG3;
};
static struct DMAMUX *const DMAMUX = (struct DMAMUX *)0x40021000;

#define INTR_REQ_DMA0 0

struct DMA {
  volatile uint32_t SAR;
  volatile uint32_t DAR;
  volatile uint32_t DSR_BCR;
  volatile uint32_t DCR;
};
static struct DMA *const DMA0 = (struct DMA *)0x40008100;
static struct DMA *const DMA1 = (struct DMA *)0x40008110;


/* We don't use ADC interrupt.  Just for reference.  */
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
static struct ADC *const ADC0 = (struct ADC *)0x4003B000;

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
#define ADC_ADLSMP       ADC_ADLSMP_SHORT
#define ADC_ADIV         ADC_ADIV_1
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

#define ADC_DMA_SLOT_NUM 40

/*
 * Buffer to save ADC data.
 */
uint32_t adc_buf[64];

static const uint32_t adc0_sc1_setting = ADC_SC1_TEMPSENSOR;

static chopstx_intr_t adc_intr;

struct adc_internal {
  uint32_t buf[64];
  uint8_t *p;
  int phase : 8;
  int count : 8;
};
struct adc_internal adc;

/*
 * Initialize ADC module, do calibration.
 *
 * This is called by MAIN, only once, hopefully before creating any
 * other threads (to be accurate).
 *
 * We configure ADC0 to kick DMA0, configure DMA0 to kick DMA1.
 * DMA0 records output of ADC0 to the ADC.BUF.
 * DMA1 kicks ADC0 again to get another value.
 *
 * ADC0 --[finish conversion]--> DMA0 --[Link channel 1]--> DMA1
 */
int
adc_init (void)
{
  uint32_t v;

  /* Enable ADC0 and DMAMUX clock.  */
  SIM->SCGC6 |= (1 << 27) | (1 << 1);
  /* Enable DMA clock.  */
  SIM->SCGC7 |= (1 << 8);

  /* ADC0 setting for calibration.  */
  ADC0->CFG1 = ADC_CLOCK_SOURCE | ADC_MODE | ADC_ADLSMP | ADC_ADIV | ADC_ADLPC;
  ADC0->CFG2 = ADC_ADLSTS | ADC_ADHSC | ADC_ADACKEN | ADC_MUXSEL;
  ADC0->SC2 = ADC_SC2_REFSEL_DEFAULT;
  ADC0->SC3 = ADC_SC3_CAL | ADC_SC3_CALF | ADC_SC3_AVGE | ADC_SC3_AVGS11;

  /* Wait ADC completion */
  while ((ADC0->SC1[0] & ADC_SC1_COCO) == 0)
    if ((ADC0->SC3 & ADC_SC3_CALF) != 0)
      /* Calibration failure */
      return -1;

  if ((ADC0->SC3 & ADC_SC3_CALF) != 0)
    /* Calibration failure */
    return -1;

  /* Configure PG by the calibration values.  */
  v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
  ADC0->PG = 0x8000 | (v >> 1);

  /* Configure MG by the calibration values.  */
  v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
  ADC0->MG = 0x8000 | (v >> 1);

  ADC0->SC1[0] = ADC_SC1_ADCSTOP;

  /* DMAMUX setting.  */
  DMAMUX->CHCFG0 = (1 << 7) | ADC_DMA_SLOT_NUM;

  /* DMA0 initial setting.  */
  DMA0->SAR = (uint32_t)&ADC0->R[0];

  /* DMA1 initial setting.  */
  DMA1->SAR = (uint32_t)&adc0_sc1_setting;
  DMA1->DAR = (uint32_t)&ADC0->SC1[0];
  
  chopstx_claim_irq (&adc_intr, INTR_REQ_DMA0);
  return 0;
}

/*
 * Start using ADC.
 */
void
adc_start (void)
{
  ADC0->CFG1 = ADC_CLOCK_SOURCE | ADC_MODE | ADC_ADLSMP | ADC_ADIV | ADC_ADLPC;
  ADC0->CFG2 = ADC_ADLSTS | ADC_ADHSC | ADC_ADACKEN | ADC_MUXSEL;
  ADC0->SC2 = ADC_SC2_REFSEL_DEFAULT | ADC_SC2_DMAEN;
  ADC0->SC3 = 0;
}

/*
 * Kick getting data for COUNT times.
 * Data will be saved in ADC_BUF starting at OFFSET.
 */
static void
adc_start_conversion_internal (int count)
{
  /* DMA0 setting.  */
  DMA0->DAR = (uint32_t)&adc.buf[0];
  DMA0->DSR_BCR = 4 * count;
  DMA0->DCR = (1 << 31) | (1 << 30) | (1 << 29) | (0 << 20) | (1 << 19)
            | (0 << 17) | (1 <<  7) | (2 <<  4) | (1 <<  2);

  /* Kick DMA1.  */
  DMA1->DSR_BCR = 4 * count;
  DMA1->DCR = (1 << 30) | (1 << 29) | (0 << 19) | (0 << 17) | (1 << 16) | (1 << 7);
}


/*
 * Kick getting data for COUNT times.
 * Data will be saved in ADC_BUF starting at OFFSET.
 */
void
adc_start_conversion (int offset, int count)
{
  adc.p = (uint8_t *)&adc_buf[offset];
  adc.phase = 0;
  adc.count = count;
  adc_start_conversion_internal (count);
}


static void
adc_stop_conversion (void)
{
  ADC0->SC1[0] = ADC_SC1_ADCSTOP;
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
adc_wait_completion (void)
{
  struct chx_poll_head *pd_array[1] = { (struct chx_poll_head *)&adc_intr };
  int i;

  while (1)
    {
      /* Wait DMA completion */
      chopstx_poll (NULL, 1, pd_array);

      DMA0->DSR_BCR = (1 << 24);
      DMA1->DSR_BCR = (1 << 24);

      adc_stop_conversion ();

      for (i = 0; i < adc.count; i++)
	*adc.p++ = (uint8_t)adc.buf[i];

      if (++adc.phase >= 4)
	break;

      adc_start_conversion_internal (adc.count);
    }

  return 0;
}
