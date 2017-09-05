#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

#include "usb_lld.h"
#include "tty.h"

/* For set_led */
#include "board.h"
#include "sys.h"

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

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

#define STACK_MAIN
#define STACK_PROCESS_1
#define STACK_PROCESS_2
#include "stack-def.h"
#define STACK_ADDR_PWM ((uint32_t)process1_base)
#define STACK_SIZE_PWM (sizeof process1_base)
#define STACK_ADDR_BLK ((uint32_t)process2_base)
#define STACK_SIZE_BLK (sizeof process2_base)


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

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

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
	  int size;
	  uint32_t usec;

	  usec = 3000000;	/* 3.0 seconds */
	  size = tty_recv (tty, s + 4, &usec);
	  if (size < 0)
	    break;

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
