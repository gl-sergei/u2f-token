#include <stdint.h>
#include <stdlib.h>
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

static struct GPIO *const GPIO_LED = ((struct GPIO *const) GPIO_LED_BASE);
static struct GPIO *const GPIO_OTHER = ((struct GPIO *const) GPIO_OTHER_BASE);

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;

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
scroll_led_display (uint8_t row)
{
  l_data[0] = (l_data[0] << 1) | ((row >> 0) & 1);
  l_data[1] = (l_data[1] << 1) | ((row >> 1) & 1);
  l_data[2] = (l_data[2] << 1) | ((row >> 2) & 1);
  l_data[3] = (l_data[3] << 1) | ((row >> 3) & 1);
  l_data[4] = (l_data[4] << 1) | ((row >> 4) & 1);
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

static void *
led (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
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

#define PRIO_LED 3

extern uint8_t __process1_stack_base__, __process1_stack_size__;

const uint32_t __stackaddr_led = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_led = (size_t)&__process1_stack_size__;

#define DATA55(x0,x1,x2,x3,x4) (x0<<20)|(x1<<15)|(x2<<10)|(x3<< 5)|(x4<< 0)
#define SIZE55(img) (sizeof (img) / sizeof (uint32_t))

static uint32_t l55[] = {
  DATA55 (0x08, 0x04, 0x1c, 0x00, 0x00),
  DATA55 (0x00, 0x14, 0x0c, 0x08, 0x00),
  DATA55 (0x00, 0x04, 0x14, 0x0c, 0x00),
  DATA55 (0x00, 0x08, 0x06, 0x0c, 0x00),
  DATA55 (0x00, 0x04, 0x02, 0x0e, 0x00),
  DATA55 (0x00, 0x00, 0x0a, 0x06, 0x04),
  DATA55 (0x00, 0x00, 0x02, 0x0a, 0x06),
  DATA55 (0x00, 0x00, 0x04, 0x03, 0x06),
  DATA55 (0x00, 0x00, 0x02, 0x01, 0x07),
  DATA55 (0x02, 0x00, 0x00, 0x05, 0x03),
  DATA55 (0x03, 0x00, 0x00, 0x01, 0x05),
  DATA55 (0x03, 0x00, 0x00, 0x02, 0x11),
  DATA55 (0x13, 0x00, 0x00, 0x01, 0x10),
  DATA55 (0x11, 0x01, 0x00, 0x00, 0x12),
  DATA55 (0x12, 0x11, 0x00, 0x00, 0x10),
  DATA55 (0x18, 0x11, 0x00, 0x00, 0x01),
  DATA55 (0x08, 0x19, 0x00, 0x00, 0x10),
  DATA55 (0x09, 0x18, 0x10, 0x00, 0x00),
  DATA55 (0x08, 0x09, 0x18, 0x00, 0x00),
  DATA55 (0x10, 0x0c, 0x18, 0x00, 0x00),
};

static uint32_t gnu[] = {
  DATA55 (0x00,	0x00, 0x00, 0x00, 0x00),
  DATA55 (0x00,	0x01, 0x01, 0x01, 0x00),
  DATA55 (0x01,	0x02, 0x02, 0x02, 0x01),
  DATA55 (0x03,	0x04, 0x04, 0x04, 0x03),
  DATA55 (0x07,	0x08, 0x09, 0x08, 0x07),
  DATA55 (0x0e,	0x10, 0x13, 0x11, 0x0e),
  DATA55 (0x1c,	0x00, 0x06, 0x02, 0x1c),
  DATA55 (0x19,	0x01, 0x0d, 0x05, 0x19),
  DATA55 (0x12,	0x03, 0x1a, 0x0a, 0x12),
  DATA55 (0x04,	0x06, 0x15, 0x14, 0x04),
  DATA55 (0x08,	0x0c, 0x0a, 0x09, 0x08),
  DATA55 (0x11,	0x19, 0x15, 0x13, 0x11),
  DATA55 (0x02,	0x12, 0x0a, 0x06, 0x02),
  DATA55 (0x05,	0x05, 0x15, 0x0d, 0x04),
  DATA55 (0x0a,	0x0a, 0x0a, 0x1a, 0x09),
  DATA55 (0x14,	0x14, 0x14, 0x14, 0x13),
  DATA55 (0x08,	0x08, 0x08, 0x08, 0x07),
  DATA55 (0x11,	0x11, 0x11, 0x11, 0x0e),
  DATA55 (0x02,	0x02, 0x02, 0x02, 0x1c),
  DATA55 (0x04,	0x04, 0x04, 0x04, 0x18),
  DATA55 (0x08,	0x08, 0x08, 0x08, 0x10),
  DATA55 (0x11,	0x11, 0x11, 0x10, 0x01),
  DATA55 (0x03,	0x03, 0x03, 0x00, 0x02),
  DATA55 (0x06,	0x06, 0x06, 0x00, 0x04),
  DATA55 (0x0c,	0x0c, 0x0c, 0x00, 0x08),
  DATA55 (0x18,	0x18, 0x18, 0x00, 0x10),
  DATA55 (0x10,	0x10, 0x10, 0x00, 0x00),
  DATA55 (0x00,	0x00, 0x00, 0x00, 0x00),
  DATA55 (0x00,	0x00, 0x00, 0x00, 0x00),
  DATA55 (0x00,	0x00, 0x00, 0x00, 0x00),
};

#define DATA55V(x0,x1,x2,x3,x4) (x0<<0)|(x1<<5)|(x2<<10)|(x3<< 15)|(x4<< 20)

#define CHAR_SPC 0
#define CHAR_H   1
#define CHAR_A   2
#define CHAR_P   3
#define CHAR_Y   4
#define CHAR_C   5
#define CHAR_K   6
#define CHAR_I   7
#define CHAR_N   8
#define CHAR_G   9
#define CHAR_EXC 10

static uint8_t hh[] = {
  CHAR_H, CHAR_A, CHAR_P, CHAR_P, CHAR_Y,
  CHAR_SPC,
  CHAR_H, CHAR_A, CHAR_C, CHAR_K, CHAR_I, CHAR_N, CHAR_G,
  CHAR_EXC,
  CHAR_SPC, CHAR_SPC, CHAR_SPC,
};

struct { uint8_t width; uint32_t data; } chargen[] = {
  { 5, 0 },						/* SPACE */
  { 5, DATA55V (0x1f, 0x04, 0x04, 0x04, 0x1f) },	/* H */
  { 5, DATA55V (0x07, 0x0a, 0x12, 0x0a, 0x07) },	/* A */
  { 5, DATA55V (0x1f, 0x14, 0x14, 0x14, 0x18) },	/* P */
  { 5, DATA55V (0x18, 0x04, 0x07, 0x04, 0x18) },	/* Y */
  { 5, DATA55V (0x0e, 0x11, 0x11, 0x11, 0x02) },	/* C */
  { 5, DATA55V (0x1f, 0x04, 0x0c, 0x12, 0x01) },	/* K */
  { 3, DATA55V (0x11, 0x1f, 0x11, 0x00, 0x00) },	/* I */
  { 5, DATA55V (0x1f, 0x08, 0x04, 0x02, 0x1f) },	/* N */
  { 5, DATA55V (0x0e, 0x11, 0x11, 0x15, 0x06) },	/* G */
  { 2, DATA55V (0x1d, 0x1c, 0x00, 0x00, 0x00) },	/* ! */
};

static uint8_t state = 0;

#define CHECK_USER() if (user_button ()) state = 0

static void
happy_hacking (void)
{
  unsigned int i, j;

  set_led_display (0);
  for (i = 0; i < sizeof (hh); i++)
    {
      for (j = 0; j < chargen[hh[i]].width; j++)
	{
	  CHECK_USER ();
	  scroll_led_display ((chargen[hh[i]].data >> j * 5) & 0x1f);
	  wait_for (150*1000);
	}

      CHECK_USER ();
      scroll_led_display (0);
      wait_for (200*1000);
    }
}


int
main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);

  chopstx_create (PRIO_LED, __stackaddr_led, __stacksize_led, led, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      unsigned int i;

      if (state == 0)
	for (i = 0; i < SIZE55 (l55); i++)
	  {
	    if (user_button ())
	      state = 1;
	    set_led_display (l55[i]);
	    wait_for (500*1000);
	  }
      else if (state == 1)
	for (i = 0; i < SIZE55 (gnu); i++)
	  {
	    if (user_button ())
	      state = 2;
	    set_led_display (gnu[i]);
	    wait_for (250*1000);
	  }
      else
	happy_hacking ();
    }

  return 0;
}
