#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include "board.h"

static uint8_t main_finished;

#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x08000000)

struct GPIO {
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  uint16_t dummy0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  uint16_t dummy1;
  volatile uint16_t ODR;
  uint16_t dummy2;
  volatile uint16_t BSRR;
  uint16_t dummy3;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint16_t BRR;
  uint16_t dummy4;
};
#define GPIOA_BASE	(AHB2PERIPH_BASE + 0x0000)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOF_BASE	(AHB2PERIPH_BASE + 0x1400)
#define GPIOF		((struct GPIO *) GPIOF_BASE)

static struct GPIO *const GPIO_LED = ((struct GPIO *const) GPIO_LED_BASE);
static struct GPIO *const GPIO_OTHER = ((struct GPIO *const) GPIO_OTHER_BASE);

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0, cnd1;

#define BUTTON_PUSHED 1
static uint8_t button_state;

static uint8_t
user_button (void)
{
  return button_state;
}


static uint8_t l_data[5];
#define LED_FULL ((0x1f << 20)|(0x1f << 15)|(0x1f << 10)|(0x1f << 5)|0x1f)

static void
set_led_display (uint32_t data)
{
  l_data[0] = (data >>  0) & 0x1f;
  l_data[1] = (data >>  5) & 0x1f;
  l_data[2] = (data >> 10) & 0x1f;
  l_data[3] = (data >> 15) & 0x1f;
  l_data[4] = (data >> 20) & 0x1f;
}

static void
scroll_led_display (uint8_t row)
{
  l_data[0] = (l_data[0] << 1) | ((row >> 0) & 1);
  l_data[1] = (l_data[1] << 1) | ((row >> 1) & 1);
  l_data[2] = (l_data[2] << 1) | ((row >> 2) & 1);
  l_data[3] = (l_data[3] << 1) | ((row >> 3) & 1);
  l_data[4] = (l_data[4] << 1) | ((row >> 4) & 1);
}


static void
wait_for (uint32_t usec)
{
  chopstx_usec_wait (usec);
}

static void
led_prepare_row (uint8_t col)
{
  uint16_t data = 0x1f;

  data |= ((l_data[0] & (1 << col)) ? 1 : 0) << 5;
  data |= ((l_data[1] & (1 << col)) ? 1 : 0) << 6;
  data |= ((l_data[2] & (1 << col)) ? 1 : 0) << 7;
  data |= ((l_data[3] & (1 << col)) ? 1 : 0) << 9;
  data |= ((l_data[4] & (1 << col)) ? 1 : 0) << 10;
  GPIO_LED->ODR = data;
}


static void
led_enable_column (uint8_t col)
{
  GPIO_LED->BRR = (1 << col);
}

static void *
led (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (!main_finished)
    {
      int i;

      for (i = 0; i < 5; i++)
	{
	  led_prepare_row (i);
	  led_enable_column (i);
	  wait_for (1000);
	}
    }

  GPIO_LED->ODR = 0x0000;	/* Off all LEDs.  */
  GPIO_LED->OSPEEDR = 0;
  GPIO_LED->OTYPER = 0;
  GPIO_LED->MODER = 0;		/* Input mode.  */
  GPIO_OTHER->PUPDR = 0x0000;	/* No pull-up.  */

  return NULL;
}


static uint8_t get_button_sw (void) { return (GPIO_OTHER->IDR & 1) == 0; }

static void *
button (void *arg)
{
  uint8_t last_button = 0;

  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd1, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (!main_finished)
    {
      uint8_t button = get_button_sw ();

      if (last_button == button && button != button_state)
	{
	  wait_for (1000);
	  button = get_button_sw ();
	  if (last_button == button)
	    button_state = button;
	}

      wait_for (2000);
      last_button = button;
    }

  return NULL;
}

#define PRIO_LED 3
#define PRIO_BUTTON 2

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;

const uint32_t __stackaddr_led = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_led = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_button = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_button = (size_t)&__process2_stack_size__;

#define DATA55(x0,x1,x2,x3,x4) (x0<<20)|(x1<<15)|(x2<<10)|(x3<< 5)|(x4<< 0)
#define SIZE55(img) (sizeof (img) / sizeof (uint32_t))

static uint32_t l55[] = {
  DATA55 (0x08, 0x04, 0x1c, 0x00, 0x00),
  DATA55 (0x00, 0x14, 0x0c, 0x08, 0x00),
  DATA55 (0x00, 0x04, 0x14, 0x0c, 0x00),
  DATA55 (0x00, 0x08, 0x06, 0x0c, 0x00),
  DATA55 (0x00, 0x04, 0x02, 0x0e, 0x00),
  DATA55 (0x00, 0x00, 0x0a, 0x06, 0x04),
  DATA55 (0x00, 0x00, 0x02, 0x0a, 0x06),
  DATA55 (0x00, 0x00, 0x04, 0x03, 0x06),
  DATA55 (0x00, 0x00, 0x02, 0x01, 0x07),
  DATA55 (0x02, 0x00, 0x00, 0x05, 0x03),
  DATA55 (0x03, 0x00, 0x00, 0x01, 0x05),
  DATA55 (0x03, 0x00, 0x00, 0x02, 0x11),
  DATA55 (0x13, 0x00, 0x00, 0x01, 0x10),
  DATA55 (0x11, 0x01, 0x00, 0x00, 0x12),
  DATA55 (0x12, 0x11, 0x00, 0x00, 0x10),
  DATA55 (0x18, 0x11, 0x00, 0x00, 0x01),
  DATA55 (0x08, 0x19, 0x00, 0x00, 0x10),
  DATA55 (0x09, 0x18, 0x10, 0x00, 0x00),
  DATA55 (0x08, 0x09, 0x18, 0x00, 0x00),
  DATA55 (0x10, 0x0c, 0x18, 0x00, 0x00),
};

#define DATA55V(x0,x1,x2,x3,x4) (x0<<0)|(x1<<5)|(x2<<10)|(x3<< 15)|(x4<< 20)

#define CHAR_SPC 0
#define CHAR_H   1
#define CHAR_A   2
#define CHAR_P   3
#define CHAR_Y   4
#define CHAR_C   5
#define CHAR_K   6
#define CHAR_I   7
#define CHAR_N   8
#define CHAR_G   9
#define CHAR_EXC 10
#define CHAR_W   11
#define CHAR_h   12
#define CHAR_t   13
#define CHAR_AP  14
#define CHAR_s   15
#define CHAR_U   16
#define CHAR_QT  17
#define CHAR_o   18
#define CHAR_X   19

static uint8_t hh[] = {
  CHAR_H, CHAR_A, CHAR_P, CHAR_P, CHAR_Y,
  CHAR_SPC,
  CHAR_H, CHAR_A, CHAR_C, CHAR_K, CHAR_I, CHAR_N, CHAR_G,
  CHAR_EXC,
  CHAR_SPC, CHAR_SPC, CHAR_SPC,
};

static uint8_t gnu[] = {
  CHAR_W, CHAR_h, CHAR_A, CHAR_t, CHAR_AP, CHAR_s, CHAR_SPC,
  CHAR_G, CHAR_N, CHAR_U, CHAR_QT, 
  CHAR_SPC, CHAR_SPC,
  CHAR_G, CHAR_N, CHAR_U, CHAR_AP, CHAR_s, CHAR_SPC,
  CHAR_N, CHAR_o, CHAR_t, CHAR_SPC,
  CHAR_U, CHAR_N, CHAR_I, CHAR_X,
  CHAR_EXC,
  CHAR_SPC, CHAR_SPC,
};

struct { uint8_t width; uint32_t data; } chargen[] = {
  { 3, 0 },						/* SPACE */
  { 4, DATA55V (0x1f, 0x04, 0x04, 0x1f, 0x00) },	/* H */
  { 3, DATA55V (0x17, 0x15, 0x0f, 0x00, 0x00) },	/* A */
  { 4, DATA55V (0x1f, 0x14, 0x14, 0x08, 0x00) },	/* P */
  { 4, DATA55V (0x19, 0x05, 0x05, 0x1e, 0x00) },	/* Y */
  { 4, DATA55V (0x0e, 0x11, 0x11, 0x0a, 0x00) },	/* C */
  { 4, DATA55V (0x1f, 0x04, 0x0c, 0x13, 0x00) },	/* K */
  { 3, DATA55V (0x11, 0x1f, 0x11, 0x00, 0x00) },	/* I */
  { 4, DATA55V (0x1f, 0x08, 0x06, 0x1f, 0x00) },	/* N */
  { 4, DATA55V (0x0e, 0x11, 0x15, 0x07, 0x00) },	/* G */
  { 2, DATA55V (0x1d, 0x1c, 0x00, 0x00, 0x00) },	/* ! */
  { 5, DATA55V (0x1e, 0x01, 0x0e, 0x01, 0x1e) },	/* W */
  { 3, DATA55V (0x1f, 0x04, 0x07, 0x00, 0x00) },	/* h */
  { 4, DATA55V (0x08, 0x1e, 0x09, 0x09, 0x00) },	/* t */
  { 3, DATA55V (0x04, 0x18, 0x18, 0x00, 0x00) },	/* ' */
  { 4, DATA55V (0x09, 0x15, 0x15, 0x12, 0x00) },	/* s */
  { 4, DATA55V (0x1e, 0x01, 0x01, 0x1e, 0x00) },	/* U */
  { 4, DATA55V (0x08, 0x10, 0x15, 0x08, 0x00) },	/* ? */
  { 4, DATA55V (0x06, 0x09, 0x09, 0x06, 0x00) },	/* o */
  { 5, DATA55V (0x11, 0x0a, 0x04, 0x0a, 0x11) },	/* X */
  { 4, DATA55V (0x1f, 0x11, 0x11, 0x0e, 0x00) },	/* D */
  { 4, DATA55V (0x0e, 0x15, 0x15, 0x0d, 0x00) },	/* e */
  { 4, DATA55V (0x1f, 0x05, 0x05, 0x06, 0x00) },	/* b */
  { 1, DATA55V (0x17, 0x00, 0x00, 0x00, 0x00) },	/* i */
  { 4, DATA55V (0x02, 0x15, 0x15, 0x0f, 0x00) },	/* a */
  { 4, DATA55V (0x0f, 0x08, 0x08, 0x0f, 0x00) },	/* n */
};


#define REPEAT_COUNT 10

static int
life_display (void)
{
  unsigned int i;
  uint8_t count = 0;
  uint8_t state = 0;

  while (count++ < REPEAT_COUNT)
    for (i = 0; i < SIZE55 (l55); i++)
      {
	if (user_button ())
	  {
	    set_led_display (LED_FULL);
	    state = 1;
	  }
	else if (state == 1)
	  return 0;
	else
	  set_led_display (l55[i]);
	wait_for (350*1000);
      }

  return 1;
}


static int
text_display (uint8_t kind)
{
  unsigned int i, j;
  uint8_t *text;
  uint8_t len;
  uint8_t count = 0;
  uint8_t state = 0;

  if (kind)
    {
      text = hh;
      len = sizeof (hh);
    }
  else
    {
      text = gnu;
      len = sizeof (gnu);
    }

  set_led_display (0);
  while (count++ < REPEAT_COUNT)
    for (i = 0; i < len; i++)
      {
	for (j = 0; j < chargen[text[i]].width; j++)
	  {
	    if (user_button ())
	      {
		set_led_display (LED_FULL);
		state = 1;
	      }
	    else if (state == 1)
	      return 0;
	    else
	      scroll_led_display ((chargen[text[i]].data >> j * 5) & 0x1f);
	    wait_for (120*1000);
	  }

	if (user_button ())
	  {
	    set_led_display (LED_FULL);
	    state = 1;
	  }
	else if (state == 1)
	  return 0;
	else
	  scroll_led_display (0);
	wait_for (120*1000);
      }

  return 1;
}


static void setup_scr_sleepdeep (void);

int
main (int argc, const char *argv[])
{
  chopstx_t led_thd;
  chopstx_t button_thd;
  uint8_t happy = 1;
  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  led_thd = chopstx_create (PRIO_LED, __stackaddr_led,
			    __stacksize_led, led, NULL);
  button_thd = chopstx_create (PRIO_BUTTON, __stackaddr_button,
			       __stacksize_button, button, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  wait_for (100*1000);
  if (user_button ())
    {
      /* Wait button release.  */
      while (user_button ())
	wait_for (100*1000);

      happy = 0;
      goto do_text;
    }

  while (1)
    {
      if (life_display ())
	break;
    do_text:
      if (text_display (happy))
	break;
    }

  main_finished = 1;
  chopstx_join (button_thd, NULL);
  chopstx_join (led_thd, NULL);

  setup_scr_sleepdeep ();
  for (;;)
    asm volatile ("wfi" : : : "memory");

  return 0;
}

struct SCB
{
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile uint32_t PFR[2];
  volatile uint32_t DFR;
  volatile uint32_t ADR;
  volatile uint32_t MMFR[4];
  volatile uint32_t ISAR[5];
};

#define SCS_BASE	(0xE000E000)
#define SCB_BASE	(SCS_BASE +  0x0D00)
static struct SCB *const SCB = ((struct SCB *const) SCB_BASE);

#define SCB_SCR_SLEEPDEEP (1 << 2)

struct PWR
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
};
#define PWR_CR_PDDS 0x0002
#define PWR_CR_CWUF 0x0004

#define PWR_BASE	(APBPERIPH_BASE + 0x00007000)
#define PWR		((struct PWR *) PWR_BASE)

static void setup_scr_sleepdeep (void)
{
  PWR->CR |= PWR_CR_CWUF;
  PWR->CR |= PWR_CR_PDDS;
  SCB->SCR |= SCB_SCR_SLEEPDEEP;
}
