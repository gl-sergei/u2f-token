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

  const uint8_t timebase = 15; /* we are using 14MHz HFRCO as HFCLK source */
  const uint8_t presc = 2;

  ADC0->CTRL = (0 << 24)           /* OVSRSEL */
               | (timebase << 16)  /* TIMEBASE */
               | (presc << 8)      /* PRESC */
               | (0 << 0)          /* WARMUPMODE */;

  ADC0->SINGLECTRL = (1 << 28)    /* PRS channel 1 (we use ch0 for capsense) */
                   | (0 << 20)    /* 1 cycle aacquisition time */
                   | (0 << 16)    /* 1.25V internal reference */
                   | (0 <<  4)    /* 12 bit resolution */
                   | (9 <<  8)    /* VDD/3 input selected */
                   | (0 <<  1)    /* differential mode */
                   | (0 << 24)    /* disable PRS */
                   | (0 <<  2)    /* right adjust */
                   | (1 <<  0);   /* repetitive mode */

  ADC0->CAL = (0x7C << 0) | (0x1F << 8);

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

  chopstx_claim_irq (&adc_intr, INTR_REQ_ADC0);

  ADC0->IFC = 0xffffffff;
  ADC0->IEN = 1; /* enable single conversion complete interrupt */
  ADC0->CMD = ADC_CMD_SINGLESTART;

  while (req.count > 0)
    {
      /* Wait for single conversion completion */
      chopstx_poll (NULL, 1, pd_array);

      if (adc_intr.ready && (ADC0->IF & 1))
        {
          adc_buf[req.offset++] = ADC0->SINGLEDATA;
          --req.count;
        }
    }

  ADC0->IEN = 0;
  ADC0->CMD = ADC_CMD_SINGLESTOP;

  return 0;
}
