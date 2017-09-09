/*
 * sys-efm32.c - First pages for EFM32.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include "board.h"

extern uint8_t __ram_end__;
void reset (void);

static uint32_t
stack_entry[] __attribute__ ((section(".first_page.first_words"),used)) = {
  (uint32_t)&__ram_end__,
  (uint32_t)reset,
};

typedef void (*handler)(void);
extern handler vector_table[];

#define ADDR_VECTORS ((uint32_t)(&vector_table))
#define ADDR_SCR_VTOR 0xe000ed08

void
reset (void)
{
  uint32_t r3 = ADDR_SCR_VTOR;

  asm volatile ("str  %2, [%0]\n\t"   /* Set SCR->VTOR     */
    "ldr  %0, [%2]\n\t"     /* Stack address     */
    "msr  MSP, %0\n\t"    /* Exception handler stack. */
    "ldr  %0, [%2, #4]\n\t" /* The entry address */
    "bx %0\n\t"           /* Jump to the entry */
    ".align 2"
    : "=r" (r3)
    : "0" (r3), "r" (ADDR_VECTORS)
    : "memory");

  /* Never reach here. */
}

#include "mcu/clk_gpio_init-efm32.c"

static void
set_led (int on)
{
  /* PA0 */
  const uint8_t pin = 0;
  const uint8_t port = 0;
  if (on)
    GPIO->P[port].DOUTCLR = 1 << pin; /* PA0: Clear: Light on  */
  else
    GPIO->P[port].DOUTSET = 1 << pin; /* PA0: Set: Light off  */
}

handler sys_vector[] __attribute__ ((section(".sys.vectors"))) = {
  clock_init,
  gpio_init,
  (handler)set_led,
  NULL,
};
