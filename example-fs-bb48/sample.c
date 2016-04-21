#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

#include "usb_lld.h"
#include "stream.h"
#include "board.h"

#include "crc32.h"

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

#define INTR_REQ_USB 24

static void *
usb_intr (void *arg)
{
  extern void usb_lld_init (uint8_t feature);
  extern void usb_interrupt_handler (void);

  chopstx_intr_t interrupt;

  (void)arg;
  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  usb_lld_init (0x80);		/* Bus powered. */

  while (1)
    {
      chopstx_intr_wait (&interrupt);

      /* Process interrupt. */
      usb_interrupt_handler ();
    }

  chopstx_release_irq (&interrupt);
  return NULL;
}

#if defined(BUSY_LOOP)
#define PRIO_PWM (CHOPSTX_SCHED_RR|1)
#define PRIO_BLK (CHOPSTX_SCHED_RR|1)
#else
#define PRIO_PWM 3
#define PRIO_BLK 2
#endif
#define PRIO_INTR 4

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;
extern uint8_t __process3_stack_base__, __process3_stack_size__;

const uint32_t __stackaddr_pwm = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_pwm = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_blk = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_blk = (size_t)&__process2_stack_size__;

const uint32_t __stackaddr_intr = (uint32_t)&__process3_stack_base__;
const size_t __stacksize_intr = (size_t)&__process3_stack_size__;


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

static struct stream *st;

static int
check_recv (void *arg)
{
  struct stream *s = arg;
  if ((s->flags & FLAG_CONNECTED) == 0)
    return 1;
  if ((s->flags & FLAG_RECV_AVAIL))
    return 1;
  return 0;
}

static void
poll_for_stream (int reg_or_unreg, chopstx_px_t *px)
{
  if (reg_or_unreg == 0)
    chopstx_cond_hook (px, &st->cnd, &st->mtx, check_recv, st);
  else
    chopstx_cond_unhook (px, &st->cnd);
}

int
main (int argc, const char *argv[])
{
  uint8_t count;
  extern uint32_t bDeviceState;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  st = stream_open ();

  m = 10;

  chopstx_create (PRIO_PWM, __stackaddr_pwm, __stacksize_pwm, pwm, NULL);
  chopstx_create (PRIO_BLK, __stackaddr_blk, __stacksize_blk, blk, NULL);
  chopstx_create (PRIO_INTR, __stackaddr_intr, __stacksize_intr,
		  usb_intr, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  u = 1;
  while (bDeviceState != CONFIGURED)
    chopstx_usec_wait (500*1000);

  count = 0;
  while (1)
    {
      uint8_t s[64];

      if (stream_wait_connection (st) < 0)
	{
	  chopstx_usec_wait (1000*1000);
	  continue;
	}

      chopstx_usec_wait (500*1000);

      /* Send ZLP at the beginning.  */
      stream_send (st, s, 0);

      memcpy (s, "xx: Hello, World with Chopstx!\r\n", 32);
      s[0] = hexchar (count >> 4);
      s[1] = hexchar (count & 0x0f);
      count++;

      if (stream_send (st, s, 32) < 0)
	continue;

      while (1)
	{
	  int size;
	  uint32_t usec;

	  /* With chopstx_poll, we can do timed cond_wait */
	  usec = 3000000;
	  if (chopstx_poll (&usec, 1, poll_for_stream))
	    {
	      size = stream_recv (st, s + 4);

	      if (size < 0)
		break;

	      if (size >= 0)
		{
		  int i;
		  unsigned int value;

		  crc32_init ();
		  s[0] = hexchar (size >> 4);
		  s[1] = hexchar (size & 0x0f);

		  for (i = 0; i < size; i++)
		    crc32_u8 (s[4 + i]);
		  value = crc32_value () ^ 0xffffffff;
		  s[4] = hexchar (value >> 28);
		  s[5] = hexchar (value >> 24);
		  s[6] = hexchar (value >> 20);
		  s[7] = hexchar (value >> 16);
		  s[8] = hexchar (value >> 12);
		  s[9] = hexchar (value >> 8);
		  s[10] = hexchar (value >> 4);
		  s[11] = hexchar (value);
		  s[12] = '\r';
		  s[13] = '\n';
		  if (stream_send (st, s, 14) < 0)
		    break;
		}
	    }

	  u ^= 1;
	}
    }

  return 0;
}
