/*
 * u2f.c - main
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware for STM32
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chopstx.h>

/* For set_led */
#include "board.h"
#include "sys.h"

#include "usb_lld.h"

#include "usb-hid.h"
#include "u2f-hid.h"
#include "random.h"
#include "adc.h"

#if defined(HAVE_PUSH_BUTTON)
#include "pbt.h"
#elif defined(HAVE_CAPSENSE)
#include "csn.h"
#else
#include "uvoid.h"
#endif

#include "platform.h"

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

uint8_t v;
uint8_t blink_is_on;

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
      chopstx_usec_wait (10*1000);
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
      chopstx_usec_wait (50*1000);
      v = 1;
      chopstx_usec_wait (50*1000);
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

#define PRIO_PWM 7
#define PRIO_BLK 7

extern uint8_t __process1_stack_base__[], __process1_stack_size__[];
extern uint8_t __process2_stack_base__[], __process2_stack_size__[];

#define STACK_ADDR_PWM ((uint32_t)__process1_stack_base__)
#define STACK_SIZE_PWM ((uint32_t)__process1_stack_size__)

#define STACK_ADDR_BLK ((uint32_t)__process2_stack_base__)
#define STACK_SIZE_BLK ((uint32_t)__process2_stack_size__)


int
main (int argc, const char *argv[])
{
  struct usb_hid *hid;

  (void)argc;
  (void)argv;

  platform_init ();

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  blink_is_on = 1;

  adc_init ();

  random_init ();

#if defined(HAVE_PUSH_BUTTON)
  pbt_init ();
#elif defined(HAVE_CAPSENSE)
  capsense_init ();
#else
  uvoid_init ();
#endif

  flash_unlock ();

  chopstx_usec_wait (200*1000);

  hid = hid_open ();
  u2f_hid_open (hid);

  while (1)
    {
      chopstx_usec_wait (500*1000);
    }

  random_fini ();

  return 0;
}
