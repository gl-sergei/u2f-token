/*
 * pbt.c - push button driver and user presence indicator
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

#include "board.h"
#include "sys.h"

#include <mcu/stm32f103.h>

#define PRIO_PBT 2

extern uint8_t __process6_stack_base__[], __process6_stack_size__[];

#define STACK_ADDR_PBT ((uint32_t)__process6_stack_base__)
#define STACK_SIZE_PBT ((uint32_t)__process6_stack_size__)

/* chopstx does not expose generic GPIO interface */

struct GPIO {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
};

#define GPIOA_BASE  (APB2PERIPH_BASE + 0x0800)
#define GPIOA   ((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE  (APB2PERIPH_BASE + 0x0C00)
#define GPIOB   ((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE  (APB2PERIPH_BASE + 0x1000)
#define GPIOC   ((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE  (APB2PERIPH_BASE + 0x1400)
#define GPIOD   ((struct GPIO *) GPIOD_BASE)
#define GPIOE_BASE  (APB2PERIPH_BASE + 0x1800)
#define GPIOE   ((struct GPIO *) GPIOE_BASE)

#if defined(TARGET_MAPLE_MINI)
#define GPIO_PBT_RD    8
#define GPIO_PBT_BASE  GPIOB_BASE
#define GPIO_PBT_IS_LO 0
#elif defined(TARGET_ST_DONGLE)
#define GPIO_PBT_RD    5
#define GPIO_PBT_BASE  GPIOA_BASE
#define GPIO_PBT_IS_LO 1
#endif

#ifdef GPIO_PBT_BASE
static struct GPIO *const GPIO_PBT = (struct GPIO *)GPIO_PBT_BASE;
#endif

static int touch = 0;

static int
pbt_get (void)
{
  return ((GPIO_PBT->IDR >> GPIO_PBT_RD) & 1) ^ GPIO_PBT_IS_LO;
}

static chopstx_intr_t intr;

static void *
pbt (void *arg)
{
  (void)arg;
  uint32_t timeout;
  uint32_t count = 0;
  int since_last_touch = 0;

  while (1)
    {
      timeout = 100 * 1000;  /* 0.1 sec */
      struct chx_poll_head *pd_array[1] = {
        (struct chx_poll_head *)&intr
      };
      chopstx_poll (&timeout, 1, pd_array);

      if (!intr.ready && count > 0)
        --count;

      if (pbt_get ())
        {
          if (since_last_touch > 5)
            {
              touch ^= 1;
              count = 100;       /* remember user presence for 10 seconds */
            }
          since_last_touch = 0;
        }
        else
          ++since_last_touch;

        if (since_last_touch > 1000)
          since_last_touch = 1000;

      if (count == 0)
        touch = 0;
    }

  return NULL;
}

int
user_presence_get (void)
{
  return touch;
}

void
user_presence_reset (void)
{
  touch = 0;
}

void
pbt_init (void)
{
  const uint32_t line = GPIO_PBT_RD;

  /* EXTIx[3:0]: EXTI x configuration (x= 8) - 0001: PB[x] pin */
  AFIO->EXTICR[2] |= 1;
  EXTI->IMR |= (1 << line);
  EXTI->FTSR |= (1 << line);

  chopstx_claim_irq (&intr, line);

  chopstx_create (PRIO_PBT, STACK_ADDR_PBT, STACK_SIZE_PBT, pbt, NULL);
}
