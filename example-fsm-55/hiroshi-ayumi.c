#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

#include "board.h"

#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x08000000)

struct GPIO {
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  uint16_t dummy0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  uint16_t dummy1;
  volatile uint16_t ODR;
  uint16_t dummy2;
  volatile uint16_t BSRR;
  uint16_t dummy3;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint16_t BRR;
  uint16_t dummy4;
};
#define GPIOA_BASE	(AHB2PERIPH_BASE + 0x0000)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOF_BASE	(AHB2PERIPH_BASE + 0x1400)
#define GPIOF		((struct GPIO *) GPIOF_BASE)

#define GPIO_SPEAKER_PIN    1

static struct GPIO *const GPIO_LED = ((struct GPIO *)GPIO_LED_BASE);
static struct GPIO *const GPIO_OTHER = ((struct GPIO *)GPIO_OTHER_BASE);

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd;

static uint8_t
user_button (void)
{
  return (GPIO_OTHER->IDR & 1) == 0;
}

static uint8_t l_data[5];

static void
set_led_display (uint32_t data)
{
  l_data[0] = (data >>  0) & 0x1f;
  l_data[1] = (data >>  5) & 0x1f;
  l_data[2] = (data >> 10) & 0x1f;
  l_data[3] = (data >> 15) & 0x1f;
  l_data[4] = (data >> 20) & 0x1f;
}


static void
wait_for (uint32_t usec)
{
  chopstx_usec_wait (usec);
}

static void
led_prepare_row (uint8_t col)
{
  uint16_t data = 0x1f;

  data |= ((l_data[0] & (1 << col)) ? 1 : 0) << 5;
  data |= ((l_data[1] & (1 << col)) ? 1 : 0) << 6;
  data |= ((l_data[2] & (1 << col)) ? 1 : 0) << 7;
  data |= ((l_data[3] & (1 << col)) ? 1 : 0) << 9;
  data |= ((l_data[4] & (1 << col)) ? 1 : 0) << 10;
  GPIO_LED->ODR = data;
}

static void
led_enable_column (uint8_t col)
{
  GPIO_LED->BRR = (1 << col);
}


#define PRIO_LED 3

extern uint8_t __process1_stack_base__[], __process1_stack_size__[];

#define STACK_ADDR_LED ((uint32_t)__process1_stack_base__)
#define STACK_SIZE_LED ((uint32_t)__process1_stack_size__)

static void *
led (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      int i;

      for (i = 0; i < 5; i++)
	{
	  led_prepare_row (i);
	  led_enable_column (i);
	  wait_for (1000);
	}
    }

  return NULL;
}


#define PRIO_SPK 4
extern uint8_t __process2_stack_base__[], __process2_stack_size__[];

#define STACK_ADDR_SPK ((uint32_t)__process2_stack_base__)
#define STACK_SIZE_SPK ((uint32_t)__process2_stack_size__)

static chopstx_mutex_t spk_mtx;
static chopstx_cond_t spk_cnd;
static chopstx_cond_t spk_cnd_no_tone;

static uint8_t tone;
#define NO_TONE 255


static uint16_t tone_table[] = {
  568, /* a = 880Hz */
  /* 536, */
  506, /* b */
  478, /* c */
  /* 451, */
  426, /* d */
  /* 402, */
  379, /* e */
  358, /* f*/
  /* 338, */
  319, /* g */
  /* 301, */
  284, /* A = 1760Hz */
  /* 268 */
  253, /* B */
  239, /* C */
};

static void
change_tone (uint8_t t)
{
  chopstx_mutex_lock (&spk_mtx);
  tone = t;
  chopstx_cond_signal (&spk_cnd_no_tone);
  chopstx_cond_wait (&spk_cnd, &spk_mtx);
  chopstx_mutex_unlock (&spk_mtx);
}

static void *
spk (void *arg)
{
  (void)arg;

  while (1)
    {
      uint8_t t;
      uint16_t w;

      chopstx_mutex_lock (&spk_mtx);
      t = tone;
      chopstx_cond_signal (&spk_cnd);
      if (t == NO_TONE)
	{
	  chopstx_cond_wait (&spk_cnd_no_tone, &spk_mtx);
	  t = tone;
	}
      chopstx_mutex_unlock (&spk_mtx);

      w = tone_table[t]*5/3;
      GPIO_OTHER->BSRR = (1 << GPIO_SPEAKER_PIN);
      wait_for (w);
      GPIO_OTHER->BRR = (1 << GPIO_SPEAKER_PIN);
      wait_for (w);
    }

  return NULL;
}


#define PRIO_MUSIC 2
extern uint8_t __process3_stack_base__[], __process3_stack_size__[];

#define STACK_ADDR_MUSIC ((uint32_t)__process3_stack_base__)
#define STACK_SIZE_MUSIC ((uint32_t)__process3_stack_size__)

#define C 0
#define D 1
#define E 2
#define F 3
#define G 4
#define A 5
#define B 6

#if 0 /* twinkle stars */
static const char *musical_score = 
  "c4c4g4g4A4A4g2f4f4e4e4d4d4c2"
  "g4g4f4f4e4e4d2g4g4f4f4e4e4d2"
  "c4c4g4g4A4A4g2f4f4e4e4d4d4c2";
#else /* tulip */
static const char *musical_score = 
  "c4d4e2c4d4e2g4e4d4c4d4e4d2"
  "c4d4e2c4d4e2g4e4d4c4d4e4c2"
  "g4g4e4g4A4A4g2e4e4d4d4c2";
#endif

static int get_t_and_l (char *tp, char *lp)
{
  static unsigned int i = 0;
  char tl, ll;

  tl = musical_score[i++];
  ll = musical_score[i++];

  if (tl >= 'a')
    *tp = tl - 'a';
  else
    *tp = tl - 'A' + 7;

  *lp = ll - '0';

  if (i >= strlen (musical_score))
    {
      i = 0;
      return 0;
    }
  else
    return 1;
}

#define WAIT_FOR_NO_TONE (1000*20)
#define WAIT_FOR_TONE (1000000*3/2)

static void *
music (void *arg)
{
  (void)arg;

  chopstx_mutex_init (&spk_mtx);
  chopstx_cond_init (&spk_cnd);
  chopstx_cond_init (&spk_cnd_no_tone);

  chopstx_create (PRIO_SPK, STACK_ADDR_SPK, STACK_SIZE_SPK, spk, NULL);

  while (1)
    {
      char t, l;
      int r;

      r = get_t_and_l (&t, &l);

      change_tone (NO_TONE);
      wait_for (WAIT_FOR_NO_TONE);
      change_tone (t);
      wait_for (WAIT_FOR_TONE / l);

      if (!r)
	{
	  change_tone (NO_TONE);
	  wait_for (WAIT_FOR_TONE * 3);
	}
    }

  return NULL;
}



#define DATA55(x0,x1,x2,x3,x4) (x0<<20)|(x1<<15)|(x2<<10)|(x3<< 5)|(x4<< 0)
#define SIZE55(img) (sizeof (img) / sizeof (uint32_t))

static uint32_t image0[] = {
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x01,0x00,0x00,0x00),
  DATA55 (0x01,0x03,0x01,0x01,0x00),
  DATA55 (0x02,0x06,0x02,0x02,0x01),
  DATA55 (0x05,0x0d,0x05,0x05,0x03),
  DATA55 (0x0b,0x1a,0x0a,0x0a,0x06),
  DATA55 (0x16,0x14,0x14,0x14,0x0c),
  DATA55 (0x0d,0x08,0x09,0x08,0x19),
  DATA55 (0x1b,0x11,0x12,0x10,0x13),
  DATA55 (0x17,0x03,0x04,0x00,0x07),
  DATA55 (0x0f,0x06,0x09,0x01,0x0e),
  DATA55 (0x1e,0x0c,0x12,0x02,0x1c),
  DATA55 (0x1d,0x19,0x05,0x05,0x18),
  DATA55 (0x1a,0x12,0x0a,0x0a,0x11),
  DATA55 (0x14,0x04,0x14,0x14,0x03),
  DATA55 (0x08,0x08,0x08,0x09,0x06),
  DATA55 (0x10,0x10,0x10,0x12,0x0c),
  DATA55 (0x00,0x00,0x00,0x04,0x18),
  DATA55 (0x00,0x00,0x00,0x08,0x10),
  DATA55 (0x00,0x00,0x00,0x10,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
};

static uint32_t image1[] = {
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x01,0x00,0x00,0x00,0x01),
  DATA55 (0x03,0x00,0x00,0x01,0x03),
  DATA55 (0x07,0x01,0x01,0x03,0x06),
  DATA55 (0x0f,0x02,0x02,0x06,0x0c),
  DATA55 (0x1f,0x05,0x05,0x0c,0x18),
  DATA55 (0x1e,0x0a,0x0a,0x18,0x11),
  DATA55 (0x1d,0x14,0x14,0x10,0x03),
  DATA55 (0x1b,0x08,0x08,0x00,0x07),
  DATA55 (0x17,0x11,0x11,0x01,0x0f),
  DATA55 (0x0e,0x02,0x02,0x02,0x1f),
  DATA55 (0x1c,0x04,0x04,0x04,0x1e),
  DATA55 (0x19,0x08,0x09,0x08,0x1d),
  DATA55 (0x13,0x10,0x13,0x10,0x1b),
  DATA55 (0x07,0x00,0x07,0x00,0x17),
  DATA55 (0x0f,0x00,0x0f,0x00,0x0f),
  DATA55 (0x1e,0x00,0x1e,0x00,0x1e),
  DATA55 (0x1c,0x00,0x1c,0x00,0x1c),
  DATA55 (0x18,0x00,0x18,0x00,0x18),
  DATA55 (0x10,0x00,0x10,0x00,0x10),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
  DATA55 (0x00,0x00,0x00,0x00,0x00),
};

int
main (int argc, const char *argv[])
{
  uint8_t state = 0;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd);

  chopstx_create (PRIO_LED, STACK_ADDR_LED, STACK_SIZE_LED, led, NULL);
  chopstx_create (PRIO_MUSIC, STACK_ADDR_MUSIC, STACK_SIZE_MUSIC, music, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      unsigned int i;

      if (state)
	for (i = 0; i < SIZE55 (image1); i++)
	  {
	    if (user_button ())
	      state = 0;
	    set_led_display (image1[i]);
	    wait_for (200*1000);
	  }
      else
	for (i = 0; i < SIZE55 (image0); i++)
	  {
	    if (user_button ())
	      state = 1;
	    set_led_display (image0[i]);
	    wait_for (200*1000);
	  }
    }

  return 0;
}
