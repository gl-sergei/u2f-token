#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

#include "usb_lld.h"

/* For set_led */
#include "board.h"
#include "sys.h"

#include "usb-hid.h"
#include "u2f-hid.h"
#include "random.h"
#include "adc.h"
#include "pbt.h"

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

uint8_t v;
uint8_t blink_is_on;
static uint8_t m;		/* 0..100 */

static void *
pwm (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      set_led ((blink_is_on & v) || user_presence_get ());
      chopstx_usec_wait (m);
      set_led (0);
      chopstx_usec_wait (100-m);
    }

  return NULL;
}

static void *
blk (void *arg)
{
  (void)arg;

  int nblk = 0;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd1, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      v = 0;
      chopstx_usec_wait (200*1000);
      v = 1;
      chopstx_usec_wait (200*1000);
      if (blink_is_on)
        nblk++;
      if (nblk == 3)
        {
          nblk = 0;
          blink_is_on = 0;
        }
    }

  return NULL;
}

#define PRIO_PWM 3
#define PRIO_BLK 2

extern uint8_t __process1_stack_base__[], __process1_stack_size__[];
extern uint8_t __process2_stack_base__[], __process2_stack_size__[];

#define STACK_ADDR_PWM ((uint32_t)__process1_stack_base__)
#define STACK_SIZE_PWM ((uint32_t)__process1_stack_size__)

#define STACK_ADDR_BLK ((uint32_t)__process2_stack_base__)
#define STACK_SIZE_BLK ((uint32_t)__process2_stack_size__)


int
main (int argc, const char *argv[])
{
  uint8_t count;
  struct usb_hid *hid;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  blink_is_on = 1;

  adc_init ();

  random_init ();

  pbt_init ();

  flash_unlock ();

  hid = hid_open ();
  u2f_hid_open (hid);

  count = 0;
  m = 50;
  while (1)
    {
      chopstx_usec_wait (50*1000);
      count++;
    }

  random_fini ();

  return 0;
}
