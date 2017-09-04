#if defined(GNU_LINUX_EMULATION)
#include "mcu/sys-gnu-linux.h"
#elif defined(MCU_KINETIS_L)
#include "mcu/sys-mkl27z.h"
#elif defined(MCU_STM32F0)
#include "mcu/sys-stm32f0.h"
#else
#include "mcu/sys-stm32f103.h"
#endif
