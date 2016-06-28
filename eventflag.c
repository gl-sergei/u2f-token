/*
 * eventflag.c - Eventflag
 *
 * Copyright (C) 2013, 2016  Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
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
 * receipents of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <eventflag.h>


void
eventflag_init (struct eventflag *ev)
{
  ev->flags = 0;
  chopstx_cond_init (&ev->cond);
  chopstx_mutex_init (&ev->mutex);
}


static int
eventflag_check (void *arg)
{
  struct eventflag *ev = arg;

  return ev->flags != 0;
}


void
eventflag_prepare_poll (struct eventflag *ev, chopstx_poll_cond_t *poll_desc)
{ 
  poll_desc->type = CHOPSTX_POLL_COND;
  poll_desc->ready = 0;
  poll_desc->cond = &ev->cond;
  poll_desc->mutex = &ev->mutex;
  poll_desc->check = eventflag_check;
  poll_desc->arg = ev;
}


eventmask_t
eventflag_get (struct eventflag *ev)
{
  int n;
  eventmask_t m;

  chopstx_mutex_lock (&ev->mutex);
  n = __builtin_ffs (ev->flags);
  if (n)
    {
      m = (1 << (n - 1));
      ev->flags &= ~m;
    }
  else
    m = 0;
  chopstx_mutex_unlock (&ev->mutex);

  return m;
}


eventmask_t
eventflag_wait (struct eventflag *ev)
{
  int n;
  eventmask_t m;

  chopstx_mutex_lock (&ev->mutex);
  if (!ev->flags)
    chopstx_cond_wait (&ev->cond, &ev->mutex);

  n = __builtin_ffs (ev->flags);
  if (n) /* Always n > 0 when waked up, but make sure no bad things.  */
    {
      m = (1 << (n - 1));
      ev->flags &= ~m;
    }
  else
    m = 0;
  chopstx_mutex_unlock (&ev->mutex);

  return m;
}


eventmask_t
eventflag_wait_timeout (struct eventflag *ev, uint32_t usec)
{
  chopstx_poll_cond_t poll_desc;
  struct chx_poll_head *pd_array[1] = { (struct chx_poll_head *)&poll_desc };

  eventflag_prepare_poll (ev, &poll_desc);
  chopstx_poll (&usec, 1, pd_array);
  return eventflag_get (ev);
}


void
eventflag_signal (struct eventflag *ev, eventmask_t m)
{
  chopstx_mutex_lock (&ev->mutex);
  ev->flags |= m;
  chopstx_cond_signal (&ev->cond);
  chopstx_mutex_unlock (&ev->mutex);
}
