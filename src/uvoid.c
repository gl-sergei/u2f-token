/*
 * uvoid.c - void user presence indicator
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

#define PRIO_UVOID 2

extern uint8_t __process6_stack_base__[], __process6_stack_size__[];

#define STACK_ADDR_UVOID ((uint32_t)__process6_stack_base__)
#define STACK_SIZE_UVOID ((uint32_t)__process6_stack_size__)

static int touch = 1;

static void *
uvoid (void *arg)
{
  (void)arg;

  int i;

  for (i = 0; i < 100; i++)
    chopstx_usec_wait (100*1000);

  touch = 0;

  while (1)
    chopstx_usec_wait (100*1000);

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
uvoid_init (void)
{
  chopstx_create (PRIO_UVOID, STACK_ADDR_UVOID, STACK_SIZE_UVOID, uvoid, NULL);
}
