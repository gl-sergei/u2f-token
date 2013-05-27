/*
 * chopstx.h - Threads and only threads.
 *
 * Copyright (C) 2013 Flying Stone Technology
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

typedef uint32_t chopstx_t;
struct struct_attr { uint32_t prio; uint32_t addr; size_t size; };
typedef struct struct_attr chopstx_attr_t;

void chopstx_attr_init (chopstx_attr_t *attr);

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_attr_setschedparam (chopstx_attr_t *attr, uint8_t prio);

void chopstx_attr_setstack (chopstx_attr_t *attr, uint32_t addr,
			    size_t size);

void chopstx_create (chopstx_t *thd, const chopstx_attr_t *attr,
		     void *(thread_entry) (void *), void *);

void chopstx_usleep (uint32_t useconds);

struct chx_spinlock {
  /* nothing for uniprocessor.  */
};

typedef uint8_t chopstx_prio_t;
typedef struct chx_mtx {
  struct {
    struct chx_thread *next, *prev;
  } q;
  struct chx_spinlock lock;
  struct chx_thread *owner;
  struct chx_mtx *list;
} chopstx_mutex_t;

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_mutex_init (chopstx_mutex_t *mutex);

void chopstx_mutex_lock (chopstx_mutex_t *mutex);

void chopstx_mutex_unlock (chopstx_mutex_t *mutex);

typedef struct chx_cond {
  struct {
    struct chx_thread *next, *prev;
  } q;
  struct chx_spinlock lock;
} chopstx_cond_t;

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_cond_init (chopstx_cond_t *cond);

void chopstx_cond_wait (chopstx_cond_t *cond, chopstx_mutex_t *mutex);
void chopstx_cond_signal (chopstx_cond_t *cond);
void chopstx_cond_broadcast (chopstx_cond_t *cond);

typedef struct chx_intr {
  struct chx_intr *next;
  struct chx_spinlock lock;
  struct chx_thread *t;
  uint8_t irq_num;
  uint8_t ready;
} chopstx_intr_t;

void chopstx_claim_irq (chopstx_intr_t *intr, uint8_t irq_num);
void chopstx_release_irq (chopstx_intr_t *intr);

void chopstx_wait_intr (chopstx_intr_t *intr);

/* Library provides default as weak reference.  User can replace it.  */
void chx_fatal (uint32_t err_code);
