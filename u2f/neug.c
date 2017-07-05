/*
 * neug.c - true random number generation
 *
 * Copyright (C) 2011, 2012, 2013, 2016
 *               Free Software Initiative of Japan
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of NeuG, a True Random Number Generator
 * implementation based on quantization error of ADC (for STM32F103).
 *
 * NeuG is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NeuG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <string.h>
#include <chopstx.h>

#include "sys.h"
#include "neug.h"
#include "mcu/stm32f103.h"
#include "adc.h"
#include "sha256.h"

static chopstx_mutex_t mode_mtx;
static chopstx_cond_t  mode_cond;

static sha256_context sha256_ctx_data;
static uint32_t sha256_output[SHA256_DIGEST_SIZE/sizeof (uint32_t)];

/*
 * To be a full entropy source, the requirement is to have N samples
 * for output of 256-bit, where:
 *
 *      N = (256 * 2) / <min-entropy of a sample>
 *
 * For example, N should be more than 103 for min-entropy = 5.0.
 *
 * On the other hand, in the section 6.2 "Full Entropy Source
 * Requirements", it says:
 *
 *     At least twice the block size of the underlying cryptographic
 *     primitive shall be provided as input to the conditioning
 *     function to produce full entropy output.
 *
 * For us, cryptographic primitive is SHA-256 and its blocksize is
 * 512-bit (64-byte), thus, N >= 128.
 *
 * We chose N=140.  Note that we have "additional bits" of 16-byte for
 * last block (feedback from previous output of SHA-256) to feed
 * hash_df function of SHA-256, together with sample data of 140-byte.
 *
 * N=140 corresponds to min-entropy >= 3.68.
 *
 */
#define NUM_NOISE_INPUTS 140

#define EP_ROUND_0 0 /* initial-five-byte and 3-byte, then 56-byte-input */
#define EP_ROUND_1 1 /* 64-byte-input */
#define EP_ROUND_2 2 /* 17-byte-input */
#define EP_ROUND_RAW      3 /* 32-byte-input */
#define EP_ROUND_RAW_DATA 4 /* 32-byte-input */

#define EP_ROUND_0_INPUTS 56
#define EP_ROUND_1_INPUTS 64
#define EP_ROUND_2_INPUTS 17
#define EP_ROUND_RAW_INPUTS 32
#define EP_ROUND_RAW_DATA_INPUTS 32

static uint8_t ep_round;

static void noise_source_continuous_test (uint8_t noise);
static void noise_source_continuous_test_word (uint8_t b0, uint8_t b1,
					       uint8_t b2, uint8_t b3);

/*
 * Hash_df initial string:
 *
 *  Initial five bytes are:
 *    1,          : counter = 1
 *    0, 0, 1, 0  : no_of_bits_returned (in big endian)
 *
 *  Then, three-byte from noise source follows.
 *
 *  One-byte was used in the previous turn, and we have three bytes in
 *  CRC->DR.
 */
static void ep_fill_initial_string (void)
{
  uint32_t v = CRC->DR;
  uint8_t b1, b2, b3;

  b3 = v >> 24;
  b2 = v >> 16;
  b1 = v >> 8;

  noise_source_continuous_test (b1);
  noise_source_continuous_test (b2);
  noise_source_continuous_test (b3);

  adc_buf[0] = 0x01000001;
  adc_buf[1] = (v & 0xffffff00);
}

static void ep_init (int mode)
{
  if (mode == NEUG_MODE_RAW)
    {
      ep_round = EP_ROUND_RAW;
      adc_start_conversion (0, EP_ROUND_RAW_INPUTS);
    }
  else if (mode == NEUG_MODE_RAW_DATA)
    {
      ep_round = EP_ROUND_RAW_DATA;
      adc_start_conversion (0, EP_ROUND_RAW_DATA_INPUTS / 4);
    }
  else
    {
      ep_round = EP_ROUND_0;
      ep_fill_initial_string ();
      adc_start_conversion (2, EP_ROUND_0_INPUTS);
    }
}


static void ep_fill_wbuf_v (int i, int test, uint32_t v)
{
  if (test)
    {
      uint8_t b0, b1, b2, b3;

      b3 = v >> 24;
      b2 = v >> 16;
      b1 = v >> 8;
      b0 = v;

      noise_source_continuous_test_word (b0, b1, b2, b3);
    }

  sha256_ctx_data.wbuf[i] = v;
}

/* Here, we assume a little endian architecture.  */
static int ep_process (int mode)
{
  int i, n;
  uint32_t v;

  if (ep_round == EP_ROUND_0)
    {
      sha256_start (&sha256_ctx_data);
      sha256_ctx_data.wbuf[0] = adc_buf[0];
      sha256_ctx_data.wbuf[1] = adc_buf[1];
      for (i = 0; i < EP_ROUND_0_INPUTS / 4; i++)
	{
	  CRC->DR = adc_buf[i*4 + 2];
	  CRC->DR = adc_buf[i*4 + 3];
	  CRC->DR = adc_buf[i*4 + 4];
	  CRC->DR = adc_buf[i*4 + 5];
	  v = CRC->DR;
	  ep_fill_wbuf_v (i+2, 1, v);
	}

      adc_start_conversion (0, EP_ROUND_1_INPUTS);
      sha256_process (&sha256_ctx_data);
      ep_round++;
      return 0;
    }
  else if (ep_round == EP_ROUND_1)
    {
      for (i = 0; i < EP_ROUND_1_INPUTS / 4; i++)
	{
	  CRC->DR = adc_buf[i*4];
	  CRC->DR = adc_buf[i*4 + 1];
	  CRC->DR = adc_buf[i*4 + 2];
	  CRC->DR = adc_buf[i*4 + 3];
	  v = CRC->DR;
	  ep_fill_wbuf_v (i, 1, v);
	}

      adc_start_conversion (0, EP_ROUND_2_INPUTS + 3);
      sha256_process (&sha256_ctx_data);
      ep_round++;
      return 0;
    }
  else if (ep_round == EP_ROUND_2)
    {
      for (i = 0; i < EP_ROUND_2_INPUTS / 4; i++)
	{
	  CRC->DR = adc_buf[i*4];
	  CRC->DR = adc_buf[i*4 + 1];
	  CRC->DR = adc_buf[i*4 + 2];
	  CRC->DR = adc_buf[i*4 + 3];
	  v = CRC->DR;
	  ep_fill_wbuf_v (i, 1, v);
	}

      CRC->DR = adc_buf[i*4];
      CRC->DR = adc_buf[i*4 + 1];
      CRC->DR = adc_buf[i*4 + 2];
      CRC->DR = adc_buf[i*4 + 3];
      v = CRC->DR & 0xff;   /* First byte of CRC->DR is used here.  */
      noise_source_continuous_test (v);
      sha256_ctx_data.wbuf[i] = v;
      ep_init (NEUG_MODE_CONDITIONED); /* The rest three-byte of
					  CRC->DR is used here.  */
      n = SHA256_DIGEST_SIZE / 2;
      memcpy (((uint8_t *)sha256_ctx_data.wbuf) + EP_ROUND_2_INPUTS,
	      sha256_output, n);
      sha256_ctx_data.total[0] = 5 + NUM_NOISE_INPUTS + n;
      sha256_finish (&sha256_ctx_data, (uint8_t *)sha256_output);
      return SHA256_DIGEST_SIZE / sizeof (uint32_t);
    }
  else if (ep_round == EP_ROUND_RAW)
    {
      for (i = 0; i < EP_ROUND_RAW_INPUTS / 4; i++)
	{
	  CRC->DR = adc_buf[i*4];
	  CRC->DR = adc_buf[i*4 + 1];
	  CRC->DR = adc_buf[i*4 + 2];
	  CRC->DR = adc_buf[i*4 + 3];
	  v = CRC->DR;
	  ep_fill_wbuf_v (i, 1, v);
	}

      ep_init (mode);
      return EP_ROUND_RAW_INPUTS / 4;
    }
  else if (ep_round == EP_ROUND_RAW_DATA)
    {
      for (i = 0; i < EP_ROUND_RAW_DATA_INPUTS / 4; i++)
	{
	  v = adc_buf[i];
	  ep_fill_wbuf_v (i, 0, v);
	}

      ep_init (mode);
      return EP_ROUND_RAW_DATA_INPUTS / 4;
    }

  return 0;
}


static const uint32_t *ep_output (int mode)
{
  if (mode)
    return sha256_ctx_data.wbuf;
  else
    return sha256_output;
}

#define REPETITION_COUNT           1
#define ADAPTIVE_PROPORTION_64     2
#define ADAPTIVE_PROPORTION_4096   4

uint8_t neug_err_state;
uint16_t neug_err_cnt;
uint16_t neug_err_cnt_rc;
uint16_t neug_err_cnt_p64;
uint16_t neug_err_cnt_p4k;

uint16_t neug_rc_max;
uint16_t neug_p64_max;
uint16_t neug_p4k_max;

static void noise_source_cnt_max_reset (void)
{
  neug_err_cnt = neug_err_cnt_rc = neug_err_cnt_p64 = neug_err_cnt_p4k = 0;
  neug_rc_max = neug_p64_max = neug_p4k_max = 0;
}

static void noise_source_error_reset (void)
{
  neug_err_state = 0;
}

static void noise_source_error (uint32_t err)
{
  neug_err_state |= err;
  neug_err_cnt++;

  if ((err & REPETITION_COUNT))
    neug_err_cnt_rc++;
  if ((err & ADAPTIVE_PROPORTION_64))
    neug_err_cnt_p64++;
  if ((err & ADAPTIVE_PROPORTION_4096))
    neug_err_cnt_p4k++;
}

/*
 * For health tests, we assume that the device noise source has
 * min-entropy >= 4.2.  Observing raw data stream (before CRC-32) has
 * more than 4.2 bit/byte entropy.  When the data stream after CRC-32
 * filter will be less than 4.2 bit/byte entropy, that must be
 * something wrong.  Note that even we observe < 4.2, we still have
 * some margin, since we use NUM_NOISE_INPUTS=140.
 *
 */

/* Cuttoff = 9, when min-entropy = 4.2, W= 2^-30 */
/* ceiling of (1+30/4.2) */
#define REPITITION_COUNT_TEST_CUTOFF 9

static uint8_t rct_a;
static uint8_t rct_b;

static void repetition_count_test (uint8_t sample)
{
  if (rct_a == sample)
    {
      rct_b++;
      if (rct_b >= REPITITION_COUNT_TEST_CUTOFF)
	noise_source_error (REPETITION_COUNT);
      if (rct_b > neug_rc_max)
	neug_rc_max = rct_b;
   }
  else
    {
      rct_a = sample;
      rct_b = 1;
    }
}

static void repetition_count_test_word (uint8_t b0, uint8_t b1,
					uint8_t b2, uint8_t b3)
{
  if (rct_a == b0)
    rct_b++;
  else
    {
      rct_a = b0;
      rct_b = 1;
    }

  if (rct_a == b1)
    rct_b++;
  else
    {
      rct_a = b1;
      rct_b = 1;
    }

  if (rct_a == b2)
    rct_b++;
  else
    {
      rct_a = b2;
      rct_b = 1;
    }

  if (rct_a == b3)
    rct_b++;
  else
    {
      rct_a = b3;
      rct_b = 1;
    }

  if (rct_b >= REPITITION_COUNT_TEST_CUTOFF)
    noise_source_error (REPETITION_COUNT);
  if (rct_b > neug_rc_max)
    neug_rc_max = rct_b;
}

/* Cuttoff = 18, when min-entropy = 4.2, W= 2^-30 */
/* With R, qbinom(1-2^-30,64,2^-4.2) */
#define ADAPTIVE_PROPORTION_64_TEST_CUTOFF 18

static uint8_t ap64t_a;
static uint8_t ap64t_b;
static uint8_t ap64t_s;

static void adaptive_proportion_64_test (uint8_t sample)
{
  if (ap64t_s++ >= 64)
    {
      ap64t_a = sample;
      ap64t_s = 1;
      ap64t_b = 0;
    }
  else
    if (ap64t_a == sample)
      {
	ap64t_b++;
	if (ap64t_b > ADAPTIVE_PROPORTION_64_TEST_CUTOFF)
	  noise_source_error (ADAPTIVE_PROPORTION_64);
	if (ap64t_b > neug_p64_max)
	  neug_p64_max = ap64t_b;
      }
}

static void adaptive_proportion_64_test_word (uint8_t b0, uint8_t b1,
					      uint8_t b2, uint8_t b3)
{
  if (ap64t_s >= 64)
    {
      ap64t_a = b0;
      ap64t_s = 4;
      ap64t_b = 0;
    }
  else
    {
      ap64t_s += 4;

      if (ap64t_a == b0)
	ap64t_b++;
    }

  if (ap64t_a == b1)
    ap64t_b++;

  if (ap64t_a == b2)
    ap64t_b++;

  if (ap64t_a == b3)
    ap64t_b++;

  if (ap64t_b > ADAPTIVE_PROPORTION_64_TEST_CUTOFF)
    noise_source_error (ADAPTIVE_PROPORTION_64);
  if (ap64t_b > neug_p64_max)
    neug_p64_max = ap64t_b;
}

/* Cuttoff = 315, when min-entropy = 4.2, W= 2^-30 */
/* With R, qbinom(1-2^-30,4096,2^-4.2) */
#define ADAPTIVE_PROPORTION_4096_TEST_CUTOFF 315

static uint8_t ap4096t_a;
static uint16_t ap4096t_b;
static uint16_t ap4096t_s;

static void adaptive_proportion_4096_test (uint8_t sample)
{
  if (ap4096t_s++ >= 4096)
    {
      ap4096t_a = sample;
      ap4096t_s = 1;
      ap4096t_b = 0;
    }
  else
    if (ap4096t_a == sample)
      {
	ap4096t_b++;
	if (ap4096t_b > ADAPTIVE_PROPORTION_4096_TEST_CUTOFF)
	  noise_source_error (ADAPTIVE_PROPORTION_4096);
	if (ap4096t_b > neug_p4k_max)
	  neug_p4k_max = ap4096t_b;
      }
}

static void adaptive_proportion_4096_test_word (uint8_t b0, uint8_t b1,
						uint8_t b2, uint8_t b3)
{
  if (ap4096t_s >= 4096)
    {
      ap4096t_a = b0;
      ap4096t_s = 4;
      ap4096t_b = 0;
    }
  else
    {
      ap4096t_s += 4;

      if (ap4096t_a == b0)
	ap4096t_b++;
    }

  if (ap4096t_a == b1)
    ap4096t_b++;

  if (ap4096t_a == b2)
	ap4096t_b++;

  if (ap4096t_a == b3)
    ap4096t_b++;

  if (ap4096t_b > ADAPTIVE_PROPORTION_4096_TEST_CUTOFF)
    noise_source_error (ADAPTIVE_PROPORTION_4096);
  if (ap4096t_b > neug_p4k_max)
    neug_p4k_max = ap4096t_b;
}


static void noise_source_continuous_test (uint8_t noise)
{
  repetition_count_test (noise);
  adaptive_proportion_64_test (noise);
  adaptive_proportion_4096_test (noise);
}

static void noise_source_continuous_test_word (uint8_t b0, uint8_t b1,
					       uint8_t b2, uint8_t b3)
{
  repetition_count_test_word (b0, b1, b2, b3);
  adaptive_proportion_64_test_word (b0, b1, b2, b3);
  adaptive_proportion_4096_test_word (b0, b1, b2, b3);
}

/*
 * Ring buffer, filled by generator, consumed by neug_get routine.
 */
struct rng_rb {
  uint32_t *buf;
  chopstx_mutex_t m;
  chopstx_cond_t data_available;
  chopstx_cond_t space_available;
  uint8_t head, tail;
  uint8_t size;
  unsigned int full :1;
  unsigned int empty :1;
};

static void rb_init (struct rng_rb *rb, uint32_t *p, uint8_t size)
{
  rb->buf = p;
  rb->size = size;
  chopstx_mutex_init (&rb->m);
  chopstx_cond_init (&rb->data_available);
  chopstx_cond_init (&rb->space_available);
  rb->head = rb->tail = 0;
  rb->full = 0;
  rb->empty = 1;
}

static void rb_add (struct rng_rb *rb, uint32_t v)
{
  rb->buf[rb->tail++] = v;
  if (rb->tail == rb->size)
    rb->tail = 0;
  if (rb->tail == rb->head)
    rb->full = 1;
  rb->empty = 0;
}

static uint32_t rb_del (struct rng_rb *rb)
{
  uint32_t v = rb->buf[rb->head++];

  if (rb->head == rb->size)
    rb->head = 0;
  if (rb->head == rb->tail)
    rb->empty = 1;
  rb->full = 0;

  return v;
}

uint8_t neug_mode;
static int rng_should_terminate;
static chopstx_t rng_thread;


/**
 * @brief Random number generation thread.
 */
static void *
rng (void *arg)
{
  struct rng_rb *rb = (struct rng_rb *)arg;
  int mode = neug_mode;

  rng_should_terminate = 0;
  chopstx_mutex_init (&mode_mtx);
  chopstx_cond_init (&mode_cond);

  /* Enable ADCs */
  adc_start ();

  ep_init (mode);
  while (!rng_should_terminate)
    {
      int err;
      int n;

      err = adc_wait_completion ();

      chopstx_mutex_lock (&mode_mtx);
      if (err || mode != neug_mode)
	{
	  mode = neug_mode;

	  noise_source_cnt_max_reset ();

	  /* Discarding data available, re-initiate from the start.  */
	  ep_init (mode);
	  chopstx_cond_signal (&mode_cond);
	  chopstx_mutex_unlock (&mode_mtx);
	  continue;
	}
      else
	chopstx_mutex_unlock (&mode_mtx);

      if ((n = ep_process (mode)))
	{
	  int i;
	  const uint32_t *vp;

	  if (neug_err_state != 0
	      && (mode == NEUG_MODE_CONDITIONED || mode == NEUG_MODE_RAW))
	    {
	      /* Don't use the result and do it again.  */
	      noise_source_error_reset ();
	      continue;
	    }

	  vp = ep_output (mode);

	  chopstx_mutex_lock (&rb->m);
	  while (rb->full)
	    chopstx_cond_wait (&rb->space_available, &rb->m);

	  for (i = 0; i < n; i++)
	    {
	      rb_add (rb, *vp++);
	      if (rb->full)
		break;
	    }

	  chopstx_cond_signal (&rb->data_available);
	  chopstx_mutex_unlock (&rb->m);
	}
    }

  adc_stop ();

  return NULL;
}

static struct rng_rb the_ring_buffer;

extern uint8_t __process5_stack_base__[], __process5_stack_size__[];
#define STACK_ADDR_RNG ((uint32_t)__process5_stack_base__)
#define STACK_SIZE_RNG ((uint32_t)__process5_stack_size__)
#define PRIO_RNG 2

/**
 * @brief Initialize NeuG.
 */
void
neug_init (uint32_t *buf, uint8_t size)
{
  const uint32_t *u = (const uint32_t *)unique_device_id ();
  struct rng_rb *rb = &the_ring_buffer;
  int i;

  RCC->AHBENR |= RCC_AHBENR_CRCEN;
  CRC->CR = CRC_CR_RESET;

  /*
   * This initialization ensures that it generates different sequence
   * even if all physical conditions are same.
   */
  for (i = 0; i < 3; i++)
    CRC->DR = *u++;

  neug_mode = NEUG_MODE_CONDITIONED;
  rb_init (rb, buf, size);

  rng_thread = chopstx_create (PRIO_RNG, STACK_ADDR_RNG, STACK_SIZE_RNG,
			       rng, rb);
}

/**
 * @breif Flush random bytes.
 */
void
neug_flush (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chopstx_mutex_lock (&rb->m);
  while (!rb->empty)
    (void)rb_del (rb);
  chopstx_cond_signal (&rb->space_available);
  chopstx_mutex_unlock (&rb->m);
}


/**
 * @brief  Wakes up RNG thread to generate random numbers.
 */
void
neug_kick_filling (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chopstx_mutex_lock (&rb->m);
  if (!rb->full)
    chopstx_cond_signal (&rb->space_available);
  chopstx_mutex_unlock (&rb->m);
}

/**
 * @brief  Get random word (32-bit) from NeuG.
 * @detail With NEUG_KICK_FILLING, it wakes up RNG thread.
 *         With NEUG_NO_KICK, it doesn't wake up RNG thread automatically,
 *         it is needed to call neug_kick_filling later.
 */
uint32_t
neug_get (int kick)
{
  struct rng_rb *rb = &the_ring_buffer;
  uint32_t v;

  chopstx_mutex_lock (&rb->m);
  while (rb->empty)
    chopstx_cond_wait (&rb->data_available, &rb->m);
  v = rb_del (rb);
  if (kick)
    chopstx_cond_signal (&rb->space_available);
  chopstx_mutex_unlock (&rb->m);

  return v;
}

int
neug_get_nonblock (uint32_t *p)
{
  struct rng_rb *rb = &the_ring_buffer;
  int r = 0;

  chopstx_mutex_lock (&rb->m);
  if (rb->empty)
    {
      r = -1;
      chopstx_cond_signal (&rb->space_available);
    }
  else
    *p = rb_del (rb);
  chopstx_mutex_unlock (&rb->m);

  return r;
}

int neug_consume_random (void (*proc) (uint32_t, int))
{
  int i = 0;
  struct rng_rb *rb = &the_ring_buffer;

  chopstx_mutex_lock (&rb->m);
  while (!rb->empty)
    {
      uint32_t v;

      v = rb_del (rb);
      proc (v, i);
      i++;
    }
  chopstx_cond_signal (&rb->space_available);
  chopstx_mutex_unlock (&rb->m);

  return i;
}

void
neug_wait_full (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chopstx_mutex_lock (&rb->m);
  while (!rb->full)
    chopstx_cond_wait (&rb->data_available, &rb->m);
  chopstx_mutex_unlock (&rb->m);
}

void
neug_fini (void)
{
  rng_should_terminate = 1;
  neug_get (1);
  chopstx_join (rng_thread, NULL);
}

void
neug_mode_select (uint8_t mode)
{
  if (neug_mode == mode)
    return;

  neug_wait_full ();

  chopstx_mutex_lock (&mode_mtx);
  neug_mode = mode;
  neug_flush ();
  chopstx_cond_wait (&mode_cond, &mode_mtx);
  chopstx_mutex_unlock (&mode_mtx);

  neug_wait_full ();
  neug_flush ();
}
