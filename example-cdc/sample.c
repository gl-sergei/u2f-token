#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>
#include "sys.h" /* for set_led */
#include "usb_lld.h" /* for set_led */

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

chopstx_mutex_t usb_mtx;
chopstx_cond_t cnd_usb;

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

#define INTR_REQ_USB 20

static void *
usb_intr (void *arg)
{
  extern void usb_lld_init (uint8_t feature);
  extern void usb_interrupt_handler (void);

  chopstx_intr_t interrupt;

  (void)arg;
  asm volatile ("cpsid   i" : : : "memory");
  /* Disable because of usb_lld_init assumes interrupt handler.  */
  usb_lld_init (0x80);		/* Bus powered. */
  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  /* Enable */
  asm volatile ("cpsie   i" : : : "memory");

  while (1)
    {
      chopstx_intr_wait (&interrupt);

      /* Process interrupt. */
      usb_interrupt_handler ();
    }

  chopstx_release_irq (&interrupt);
  return NULL;
}

#define PRIO_PWM 3
#define PRIO_BLK 2
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
  uint8_t count;

  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  chopstx_mutex_init (&usb_mtx);
  chopstx_cond_init (&cnd_usb);

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

  while (1)
    {
      extern uint8_t connected;

      count= 0;
      u = 1;
      /* waiting USB connection */
      chopstx_mutex_lock (&usb_mtx);
      if (!connected)
	chopstx_cond_wait (&cnd_usb, &usb_mtx);
      chopstx_mutex_unlock (&usb_mtx);

      while (1)
	{
	  char s[32];

	  u ^= 1;
	  chopstx_usec_wait (200*1000*6);

	  memcpy (s, "xx: Hello, World with Chopstx!\r\n", 32);
	  s[0] = hexchar (count >> 4);
	  s[1] = hexchar (count & 0x0f);
	  count++;

	  chopstx_mutex_lock (&usb_mtx);
	  if (connected)
	    {
	      usb_lld_write (ENDP1, s, 32);
	      chopstx_cond_wait (&cnd_usb, &usb_mtx);
	    }
	  else
	    break;
	  chopstx_mutex_unlock (&usb_mtx);
	}
      chopstx_mutex_unlock (&usb_mtx);
    }

  return 0;
}
