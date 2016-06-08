#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

#include "usb_lld.h"
#include "tty.h"
#include "board.h"
#include "command.h"

struct GPIO {
  volatile uint32_t PDOR; /* Port Data Output Register    */
  volatile uint32_t PSOR; /* Port Set Output Register     */
  volatile uint32_t PCOR; /* Port Clear Output Register   */
  volatile uint32_t PTOR; /* Port Toggle Output Register  */
  volatile uint32_t PDIR; /* Port Data Input Register     */
  volatile uint32_t PDDR; /* Port Data Direction Register */
};
static struct GPIO *const GPIOB = (struct GPIO *const)0x400FF040;
static struct GPIO *const GPIOD = (struct GPIO *const)0x400FF0C0;
static struct GPIO *const GPIOE = (struct GPIO *const)0x400FF100;

static void
set_led (int on)
{
  if (on)
    GPIOB->PCOR = (1 << 0); /* PTB0: Clear: Light on  */
  else
    GPIOB->PSOR = (1 << 0); /* PTB0: Set  : Light off */
}

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

uint8_t u;
static uint8_t v;
static uint8_t m;		/* 0..100 */

static void
wait_for (uint32_t usec)
{
#if defined(BUSY_LOOP)
  uint32_t count = usec * 6;
  uint32_t i;

  for (i = 0; i < count; i++)
    asm volatile ("" : : "r" (i) : "memory");
#else
  chopstx_usec_wait (usec);
#endif
}

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
      wait_for (m);
      set_led (0);
      wait_for (100-m);
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
      wait_for (200*1000);
      v = 1;
      wait_for (200*1000);
    }

  return NULL;
}


#define PRIO_PWM 3
#define PRIO_BLK 2

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;

const uint32_t __stackaddr_pwm = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_pwm = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_blk = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_blk = (size_t)&__process2_stack_size__;


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

  m = 10;

  chopstx_create (PRIO_PWM, __stackaddr_pwm, __stacksize_pwm, pwm, NULL);
  chopstx_create (PRIO_BLK, __stackaddr_blk, __stacksize_blk, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  u = 1;

  tty = tty_open ();
  tty_wait_configured (tty);

  count = 0;
  m = 50;
  while (1)
    {
      char s[LINEBUFSIZE];

    connection_loop:
      u = 1;
      tty_wait_connection (tty);

      chopstx_usec_wait (50*1000);

      /* Send ZLP at the beginning.  */
      tty_send (tty, s, 0);

      memcpy (s, "xx: Hello, World with Chopstx!\r\n", 32);
      s[0] = hexchar (count >> 4);
      s[1] = hexchar (count & 0x0f);
      count++;

      if (tty_send (tty, s, 32) < 0)
	continue;

      while (1)
	{
	  uint32_t usec;

	  /* Prompt */
	  if (tty_send (tty, "> ", 2) < 0)
	    break;

	  usec = 3000000;	/* 3.0 seconds */
	  while (1)
	    {
	      int size = tty_recv (tty, s, &usec);
	      u ^= 1;

	      if (size < 0)
		goto connection_loop;

	      if (size == 1)
		/* Do nothing but prompt again.  */
		break;
	      else if (size)
		{
		  /* Newline into NUL */
		  s[size - 1] = 0;
		  cmd_dispatch (tty, (char *)s);
		  break;
		}
	    }
	}
    }

  return 0;
}
