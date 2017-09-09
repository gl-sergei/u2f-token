#include <mcu/efm32.h>

static void
clock_init (void)
{
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;
  CMU->HFPERCLKDIV = 0x0;
}

static void
gpio_init (void)
{
  GPIO->P[0].DOUT = 0xffffffff;
  GPIO->P[1].DOUT = 0xffffffff;
  GPIO->P[0].MODEL = 0x88888888;
  GPIO->P[0].MODEH = 0x88888888;
  GPIO->P[1].MODEL = 0x88888888;
  GPIO->P[1].MODEH = 0x88888888;
}
