#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "stm32f103.h"
#include "st7732.h"
#include "primer2-ts.h"

/* ADC3 routines.  */

#define ADC3_BASE		(APB2PERIPH_BASE + 0x3c00)
static struct ADC *const ADC3 = (struct ADC *const)ADC3_BASE;

#define RCC_APB2ENR_ADC3EN	0x8000
#define RCC_APB2RSTR_ADC3RST	0x8000

#define ADC_SR_JEOC		0x0004

#define ADC_CR1_JEOCIE		(1 << 7)

#define ADC_CR2_JSWSTART	(1 << 21)
#define ADC_CR2_JEXTTRIG        (1 << 15)
#define ADC_CR2_JEXTSEL(n)	((n) << 12)

#define ADC_JSQR_NUM_CH(n)      (((n) - 1) << 20)
#define ADC_JSQR_JSQ1_N(n)      ((n) << 0)
#define ADC_JSQR_JSQ2_N(n)      ((n) << 5)
#define ADC_JSQR_JSQ3_N(n)      ((n) << 10)
#define ADC_JSQR_JSQ4_N(n)      ((n) << 15)

#define ADC_CHANNEL_IN10        10
#define ADC_CHANNEL_IN11        11
#define ADC_CHANNEL_IN12        12
#define ADC_CHANNEL_IN13        13

#define USE_ADC3_INTR		1
#define INTR_REQ_ADC3		47

/*
 * Do calibration for ADC3.
 */
void adc3_init (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
  RCC->APB2RSTR = RCC_APB2RSTR_ADC3RST;
  RCC->APB2RSTR = 0;

  ADC3->CR1 = 0;
  ADC3->CR2 = ADC_CR2_ADON;
  ADC3->CR2 = ADC_CR2_ADON | ADC_CR2_RSTCAL;
  while ((ADC3->CR2 & ADC_CR2_RSTCAL) != 0)
    ;
  ADC3->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;
  while ((ADC3->CR2 & ADC_CR2_CAL) != 0)
    ;
  ADC3->CR2 = 0;
  RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
}

static chopstx_intr_t adc3_intr;

void
adc3_start (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

#if USE_ADC3_INTR
  ADC3->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
#else
  ADC3->CR1 = ADC_CR1_SCAN;
#endif
  ADC3->CR2 = ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL(7) | ADC_CR2_ADON;
  ADC3->SMPR1 = 0xfff;
  ADC3->SMPR2 = 0;
  ADC3->JSQR = (ADC_JSQR_NUM_CH(4) | ADC_JSQR_JSQ4_N(ADC_CHANNEL_IN13)
		| ADC_JSQR_JSQ3_N(ADC_CHANNEL_IN12)
		| ADC_JSQR_JSQ2_N(ADC_CHANNEL_IN11)
		| ADC_JSQR_JSQ1_N(ADC_CHANNEL_IN10));

#if USE_ADC3_INTR
  chopstx_claim_irq (&adc3_intr, INTR_REQ_ADC3);
#endif
}

void adc3_conversion (uint32_t *result)
{

  /* Start conversion.  */
  ADC3->CR2 |= ADC_CR2_JSWSTART;

#if USE_ADC3_INTR
  chopstx_intr_wait (&adc3_intr);
#else
  while (1)
    {
      chopstx_usec_wait (1000);
      if (ADC3->SR & ADC_SR_JEOC)
	break;
    }
#endif
  ADC3->SR &= ~ADC_SR_JEOC;

  result[0] = ADC3->JDR1;
  result[1] = ADC3->JDR2;
  result[2] = ADC3->JDR3;
  result[3] = ADC3->JDR4;

  /* Stop conversion.  */
  ADC3->CR2 &= ~ADC_CR2_JSWSTART;

  return;
}

void adc3_stop (void)
{

  /* Power off.  */
  ADC3->CR1 = 0;
  ADC3->CR2 = 0;

  RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
}

/* Touch screen routines.  */

int
ts_pushed (uint32_t u)
{
  return (u < 0xc00);
}

#define FILTER_SIZE	8

static void
ts_filter (int buf[FILTER_SIZE][2], int result[2])
{
  int s0, s1;
  int i;

  s0 = 0;
  s1 = 0;
  for (i = 0; i < FILTER_SIZE; i++)
    {
      s0 += buf[i][0];
      s1 += buf[i][1];
    }

  result[0] = s0/FILTER_SIZE;
  result[1] = s1/FILTER_SIZE;
}

/* Simple model of primer2 touch screen:
    Vdd-[R1]-[Ry]-[Rp]-[Rx]-Vss
            U    D    l    R
   where R1=1k external register, Rx(resp. Ry)=resisitive component on
   X(resp. Y) film and Rp=resisitive component of contact point.
   Convert [L, R, U, D] to [Rx, Ry, Rp].  */
void
ts_conversion (uint32_t a[], int r[])
{
  int l, u, d, ir1;
  int x, y, rp;

  l = a[0] & 0xfff;
  u = a[2] & 0xfff;
  d = a[3] & 0xfff;
  ir1 = 4096 - u;
  /* r1 = 1000 */
  x = (1000 * l)/ir1;
  y = (1000 * (u - d))/ir1;
  rp = (1000 * (d - l))/ir1;
  r[0] = x;
  r[1] = y;
  r[2] = rp;
}

int
ts_adjust (int *r, int *cord)
{
  int x, y;
  static int buf[FILTER_SIZE][2];
  static int i = 0;
  static int fill = 0;

  if (!r)
    {
      i = 0;
      fill = 0;
      return 0;
    }

  /* TODO: We might need calibration.  */
  x = (LCD_COLUMN * (r[0] - 0x20))/0x100;
  y = (LCD_ROW * (0x1e0 - r[1]))/0x1c0;

  if (x < 0)
    x = 0;
  if (x >= LCD_COLUMN)
    x = LCD_COLUMN - 1;
  if (y < 0)
    y = 0;
  if (y >= LCD_ROW)
    y = LCD_ROW - 1;

  buf[i][0] = x;
  buf[i][1] = y;

  i++;
  if (i >= FILTER_SIZE)
    {
      i = 0;
      fill = 1;
    }

  if (!fill)
    return 0;

  ts_filter (buf, cord);
  return 1;
}
