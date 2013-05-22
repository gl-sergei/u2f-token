#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "sys.h" /* for set_led */

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;
static chopstx_cond_t cnd_intr;

static uint8_t u, v;
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
      set_led (u&v);
      chopstx_usleep (m);
      set_led (0);
      chopstx_usleep (100-m);
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
      chopstx_usleep (200*1000);
      v = 1;
      chopstx_usleep (200*1000);
    }

  return NULL;
}

#define INTR_REQ_0 0

static void *
intr (void *arg)
{
  (void)arg;
  chopstix_intr_t interrupt;

  chopstx_intr_register (&interrupt, INTR_REQ_0);

  while (1)
    {
      chopstx_wait_intr (&interrupt);

      /* process interrupt... */
      chopstx_usleep (200*1000);
    }

  return NULL;
}

#define PRIO_PWM 3
#define PRIO_BLK 2
#define PRIO_INTR 2

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;
extern uint8_t __process3_stack_base__, __process3_stack_size__;

const uint32_t __stackaddr_pwm = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_pwm = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_blk = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_blk = (size_t)&__process2_stack_size__;

const uint32_t __stackaddr_intr = (uint32_t)&__process3_stack_base__;
const size_t __stacksize_intr = (size_t)&__process3_stack_size__;


int
main (int argc, const char *argv[])
{
  chopstx_t thd;
  chopstx_attr_t attr;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_attr_init (&attr);
  chopstx_attr_setschedparam (&attr, PRIO_PWM);
  chopstx_attr_setstack (&attr, __stackaddr_pwm, __stacksize_pwm);

  chopstx_create (&thd, &attr, pwm, NULL);

  chopstx_attr_setschedparam (&attr, PRIO_BLK);
  chopstx_attr_setstack (&attr, __stackaddr_blk, __stacksize_blk);

  chopstx_create (&thd, &attr, blk, NULL);

  chopstx_attr_setschedparam (&attr, PRIO_INTR);
  chopstx_attr_setstack (&attr, __stackaddr_intr, __stacksize_intr);

  chopstx_create (&thd, &attr, intr, NULL);

  chopstx_usleep (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      u ^= 1;
      chopstx_usleep (200*1000*6);
    }

  return 0;
}
