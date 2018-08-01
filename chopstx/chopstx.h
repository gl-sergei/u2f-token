/*
 * chopstx.h - Threads and only threads.
 *
 * Copyright (C) 2013, 2016 Flying Stone Technology
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

struct chx_qh {
  struct chx_pq *next, *prev;
};

typedef uintptr_t chopstx_t;
typedef uint8_t chopstx_prio_t;

extern chopstx_t chopstx_main;


/* NOTE: This signature is different to PTHREAD's one.  */
chopstx_t
chopstx_create (uint32_t flags_and_prio,
		uintptr_t stack_addr, size_t stack_size,
		void *(thread_entry) (void *), void *);
#define CHOPSTX_PRIO_BITS 8
#define CHOPSTX_DETACHED 0x10000
#define CHOPSTX_SCHED_RR 0x20000

#define CHOPSTX_PRIO_INHIBIT_PREEMPTION 248

void chopstx_usec_wait (uint32_t usec);

struct chx_spinlock {
  /* nothing for uniprocessor.  */
};

typedef struct chx_mtx {
  struct chx_qh q;
  struct chx_spinlock lock;
  struct chx_thread *owner;
  struct chx_mtx *list;
} chopstx_mutex_t;

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_mutex_init (chopstx_mutex_t *mutex);

void chopstx_mutex_lock (chopstx_mutex_t *mutex);

void chopstx_mutex_unlock (chopstx_mutex_t *mutex);

typedef struct chx_cond {
  struct chx_qh q;
  struct chx_spinlock lock;
} chopstx_cond_t;

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_cond_init (chopstx_cond_t *cond);

void chopstx_cond_wait (chopstx_cond_t *cond, chopstx_mutex_t *mutex);
void chopstx_cond_signal (chopstx_cond_t *cond);
void chopstx_cond_broadcast (chopstx_cond_t *cond);

/*
 * Library provides default implementation as weak reference.
 * User can replace it.
  */
void chx_fatal (uint32_t err_code) __attribute__((__noreturn__));

int chopstx_join (chopstx_t, void **);
void chopstx_exit (void *retval) __attribute__((__noreturn__));


enum {
  CHOPSTX_ERR_NONE = 0,
  CHOPSTX_ERR_THREAD_CREATE,
  CHOPSTX_ERR_JOIN,
};

#define CHOPSTX_CANCELED ((void *) -1)

void chopstx_cancel (chopstx_t thd);
void chopstx_testcancel (void);

/* NOTE: This signature is different to PTHREAD's one.  */
int chopstx_setcancelstate (int);

typedef struct chx_cleanup {
  struct chx_cleanup *next;
  void (*routine) (void *);
  void *arg;
} chopstx_cleanup_t;

/* NOTE: This signature is different to PTHREAD's one.  */
void chopstx_cleanup_push (chopstx_cleanup_t *clp);
void chopstx_cleanup_pop (int execute);

chopstx_prio_t chopstx_setpriority (chopstx_prio_t);

enum {
  CHOPSTX_POLL_COND = 0,
  CHOPSTX_POLL_INTR,
  CHOPSTX_POLL_JOIN,
};

struct chx_poll_head {
  uint16_t type;
  uint16_t ready;
};

struct chx_poll_cond {
  uint16_t type;
  uint16_t ready;
  /**/
  chopstx_cond_t *cond;
  chopstx_mutex_t *mutex;
  int (*check) (void *);
  void *arg;
};
typedef struct chx_poll_cond chopstx_poll_cond_t;

struct chx_poll_join {
  uint16_t type;
  uint16_t ready;
  /**/
  chopstx_t thd;
};
typedef struct chx_poll_join chopstx_poll_join_t;

struct chx_intr {
  uint16_t type;
  uint16_t ready;
  /**/
  uint8_t irq_num;
};
typedef struct chx_intr chopstx_intr_t;

void chopstx_claim_irq (chopstx_intr_t *intr, uint8_t irq_num);

void chopstx_intr_wait (chopstx_intr_t *intr); /* DEPRECATED */


int chopstx_poll (uint32_t *usec_p, int n, struct chx_poll_head *pd_array[]);

#define CHOPSTX_THREAD_SIZE 64
