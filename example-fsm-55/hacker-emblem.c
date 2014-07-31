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

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

static uint8_t u;

static uint8_t l_data[5];

static void
set_led_display (uint32_t data)
{
  if (data)
    {
      l_data[0] = l_data[1] = l_data[2] = l_data[3] = l_data[4] = 0x1f;
    }
  else
    {
      l_data[0] = l_data[1] = l_data[2] = l_data[3] = l_data[4] = 0x00;
    }
}


static void
wait_for (uint32_t usec)
{
  chopstx_usec_wait (usec);
}

static void
led_prepare_column (uint8_t data)
{
  GPIO_LED->ODR = data;
}

static void
led_enable_row (uint8_t row)
{
  static const uint8_t row_to_pin[5] = { 5, 6, 7, 9, 10 };

  GPIO_LED->BSRR = (1 << row_to_pin[row]);
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
	  led_prepare_column (l_data [i]);
	  led_enable_row (i);
	  wait_for (1000);
	}
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
      set_led_display (0);
      wait_for (200*1000);
      set_led_display (u);
      wait_for (200*1000);
    }

  return NULL;
}

#define PRIO_LED 3
#define PRIO_BLK 2

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;
extern uint8_t __process3_stack_base__, __process3_stack_size__;

const uint32_t __stackaddr_led = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_led = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_blk = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_blk = (size_t)&__process2_stack_size__;



int
main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  chopstx_create (PRIO_LED, __stackaddr_led, __stacksize_led, led, NULL);
  chopstx_create (PRIO_BLK, __stackaddr_blk, __stacksize_blk, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      u ^= 1;
      wait_for (200*1000*6);
    }

  return 0;
}
