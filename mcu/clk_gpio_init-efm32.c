#include <mcu/efm32.h>

static void __attribute__((used))
clock_init (void)
{
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;
  CMU->HFPERCLKDIV = 1 << 8;

  CMU->OSCENCMD = CMU_OSCENCMD_HFRCOEN;
  while (!(CMU->STATUS & CMU_STATUS_HFRCORDY));

  CMU->CMD |= CMU_CMD_HFCLKSEL_HFRCO;
  while ((CMU->STATUS & CMU_STATUS_HFRCOSEL) == 0);
}

static void __attribute__((used))
gpio_init (void)
{
  GPIO->P[0].DOUT = 0xffffffff;
  GPIO->P[1].DOUT = 0xffffffff;
  GPIO->P[0].MODEL = 0x88888888;
  GPIO->P[0].MODEH = 0x88888888;
  GPIO->P[1].MODEL = 0x88888888;
  GPIO->P[1].MODEH = 0x88888888;
}
