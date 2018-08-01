#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>
#include <mcu/efm32hg.h>
#include "board.h"
#include "sys.h" /* for set_led */
#include "usb_lld.h"
#include "adc.h"
#include "tty.h"

#define FEATURE_BUS_POWERED 0x80

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;
static chopstx_cond_t cnd2;

static uint8_t u, v;
static uint8_t m;		/* 0..100 */
static uint8_t touch;

#define MASKED_WRITE(reg, mask, val) { (reg) = (((reg) & ~(mask)) | (val)); }

static void
set_led_pin (uint8_t port, uint8_t pin, int on)
{
  if (on)
    GPIO->P[port].DOUTCLR = 1 << pin;
  else
    GPIO->P[port].DOUTSET = 1 << pin;
}

static void
capsense_init (void)
{
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ACMP0
                      | CMU_HFPERCLKEN0_TIMER0 | CMU_HFPERCLKEN0_PRS;

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

  /*Set up TIMER0 CC1 to trigger on PRS channel 0 */
  TIMER0->CC[1].CTRL = (1 << 0)            /* MODE = INPUTCAPTURE */
                       | (0 << 16)         /* PRSSEL = PRSCH0 */
                       | (1 << 20)         /* INSEL = PRS */
                       | (2 << 26)         /* ICEVCTRL = RISING */
                       | (2 << 24);        /* ICEDGE = BOTH */

  /* Set up PRS channel 0 to trigger on ACMP0 output */
  PRS->CH[0].CTRL = (1 << 24)              /* EDSEL = POSEDGE */
                    | (2 << 16)            /* SOURCESEL = ACMP0 */
                    | (0 << 0);            /* SIGSEL = ACMP0OUT */
}

static void *
csn (void *arg)
{
  (void)arg;

  uint32_t count_max[2] = {0, 0};
  uint32_t count;
  uint32_t threshold;
  uint32_t channel;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd2, &mtx);
  chopstx_mutex_unlock (&mtx);

  touch = 0;

  while (1)
    {
      for (channel = 0; channel < 2; channel++)
        {
          /* select channel */
          MASKED_WRITE(ACMP0->INPUTSEL, 0x07, channel);

          TIMER0->CNT = 0;
          TIMER0->CMD = (1 << 0)/* START */;

          chopstx_usec_wait (10*1000);

          TIMER0->CMD = (1 << 1)/* STOP */;

          count = TIMER0->CNT;
          TIMER0->CNT = 0;

          threshold = count_max[channel];
          threshold -= count_max[channel] >> 3;
          if (count > 0 && count < threshold)
            touch |= 1 << channel;
          else
            touch &= ~(1 << channel);
          if (count > count_max[channel])
            count_max[channel] = count;
        }
    }

  return NULL;
}

static void *
pwm (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
  chopstx_mutex_unlock (&mtx);

  set_led_pin (0, 0, 0);
  set_led_pin (1, 7, 0);

  while (1)
    {
      if (!touch)
        set_led (u&v);
      chopstx_usec_wait (m);
      if (!touch)
          set_led (0);
      set_led_pin (0, 0, touch & 1);
      set_led_pin (1, 7, (touch & 2) >> 1);
      chopstx_usec_wait (100-m);
    }

  return NULL;
}

static void *
blk (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd1, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      v = 0;
      chopstx_usec_wait (200*1000);
      v = 1;
      chopstx_usec_wait (200*1000);
    }

  return NULL;
}

#define PRIO_PWM 3
#define PRIO_BLK 2
#define PRIO_CSN 2

extern uint8_t __process1_stack_base__[], __process1_stack_size__[];
extern uint8_t __process2_stack_base__[], __process2_stack_size__[];
extern uint8_t __process3_stack_base__[], __process3_stack_size__[];

#define STACK_ADDR_PWM ((uint32_t)__process1_stack_base__)
#define STACK_SIZE_PWM ((uint32_t)__process1_stack_size__)

#define STACK_ADDR_BLK ((uint32_t)__process2_stack_base__)
#define STACK_SIZE_BLK ((uint32_t)__process2_stack_size__)

#define STACK_ADDR_CSN ((uint32_t)__process3_stack_base__)
#define STACK_SIZE_CSN ((uint32_t)__process3_stack_size__)

static char hexchar (uint8_t x)
{
  x &= 0x0f;
  if (x <= 0x09)
    return '0' + x;
  else if (x <= 0x0f)
    return 'a' + x - 10;
  else
    return '?';
}

static void
adc_string (char *s)
{
  int i;

  adc_start_conversion (0, 6);
  adc_wait_completion ();

  for (i = 0; i < 6; i++)
    {
      s[i * 8 + 7] = hexchar ((adc_buf[i] >>  0) & 0xf);
      s[i * 8 + 6] = hexchar ((adc_buf[i] >>  4) & 0xf);
      s[i * 8 + 5] = hexchar ((adc_buf[i] >>  8) & 0xf);
      s[i * 8 + 4] = hexchar ((adc_buf[i] >> 12) & 0xf);
      s[i * 8 + 3] = hexchar ((adc_buf[i] >> 16) & 0xf);
      s[i * 8 + 2] = hexchar ((adc_buf[i] >> 20) & 0xf);
      s[i * 8 + 1] = hexchar ((adc_buf[i] >> 24) & 0xf);
      s[i * 8 + 0] = hexchar ((adc_buf[i] >> 28) & 0xf);
    }
  s[i * 8 + 0] = '\r';
  s[i * 8 + 1] = '\n';
}

int
main (int argc, const char *argv[])
{
  struct tty *tty;
  uint8_t count;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);
  chopstx_cond_init (&cnd2);

  m = 10;
  u = 0;

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);
  chopstx_create (PRIO_CSN, STACK_ADDR_CSN, STACK_SIZE_CSN, csn, NULL);

  capsense_init ();

  adc_init ();
  adc_start ();

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_cond_signal (&cnd2);
  chopstx_mutex_unlock (&mtx);

  u = 1;

  tty = tty_open ();
  tty_wait_configured (tty);

  count = 0;
  m = 50;
  while (1)
    {
      static char s[LINEBUFSIZE + 4] __attribute__((aligned(4)));
      char p[50];

      u = 1;
      tty_wait_connection (tty);

      chopstx_usec_wait (50*1000);

      /* Send ZLP at the beginning.  */
      tty_send (tty, s, 0);

      memcpy (s, "xx: You've got Chopstx running on Tomu board!\r\n", 47);
      s[0] = hexchar (count >> 4);
      s[1] = hexchar (count & 0x0f);
      count++;

      if (tty_send (tty, s, 47) < 0)
        continue;

      while (1)
        {
          int size;
          uint32_t usec;

          usec = 3000000; /* 3.0 seconds */
          size = tty_recv (tty, s + 4, &usec);
          if (size < 0)
            break;

          if (size == 4 && memcmp(s + 4, "adc", 3) == 0)
            {
              adc_string (p);
              if (tty_send (tty, p, 50) < 0)
                break;
            }

          if (size)
            {
              size--;

              s[0] = hexchar (size >> 4);
              s[1] = hexchar (size & 0x0f);
              s[2] = ':';
              s[3] = ' ';
              s[size + 4] = '\r';
              s[size + 5] = '\n';
              if (tty_send (tty, s, size + 6) < 0)
                break;
            }

          u ^= 1;
        }
    }

  return 0;
}
