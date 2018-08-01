#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "board.h"

#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)

struct GPIO {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
};

#define GPIOA_BASE	(APB2PERIPH_BASE + 0x0800)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE	(APB2PERIPH_BASE + 0x0C00)
#define GPIOB		((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE	(APB2PERIPH_BASE + 0x1000)
#define GPIOC		((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE	(APB2PERIPH_BASE + 0x1400)
#define GPIOD		((struct GPIO *) GPIOD_BASE)
#define GPIOE_BASE	(APB2PERIPH_BASE + 0x1800)
#define GPIOE		((struct GPIO *) GPIOE_BASE)

static struct GPIO *const GPIO_STICK = ((struct GPIO *const) GPIO_LED_BASE);
static struct GPIO *const GPIO_OTHER = ((struct GPIO *const) GPIO_OTHER_BASE);
static struct GPIO *const GPIO_OTHER1 = ((struct GPIO *const) GPIOC_BASE);
static struct GPIO *const GPIO_OTHER2 = ((struct GPIO *const) GPIOB_BASE);

#define GPIO_STICK_L	3
#define GPIO_STICK_R	4
#define GPIO_STICK_U	5
#define GPIO_STICK_D	6
#define GPIO_SHUTDOWN	13
#define GPIO_PBUTTON	8
#define GPIO_BACKLIGHT	8

void
shutdown (void)
{
  GPIO_OTHER1->BRR = (1 << GPIO_SHUTDOWN);
  GPIO_OTHER1->BSRR = (1 << GPIO_SHUTDOWN);

  while (1)
    chopstx_usec_wait (500*1000);
}

void
set_backlight (int on)
{
  if (on)
    GPIO_OTHER2->BSRR = (1 << GPIO_BACKLIGHT);
  else
    GPIO_OTHER2->BRR = (1 << GPIO_BACKLIGHT);
}

int
joystick (void)
{
  return (GPIO_STICK->IDR >> GPIO_STICK_L) & 0xf;
}

int
pbutton (void)
{
  return (GPIO_OTHER->IDR >> GPIO_PBUTTON) & 1;
}
