#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_USB_CLEAR_TO_ENABLE        9
#define GPIO_LED_SET_TO_EMIT            1

/*
 * Port B setup.
 * PB1  - Push pull output 50MHz (LED 1:ON 0:OFF)
 * PB9  - Push pull output 50MHz (USB 1:ON 0:OFF)
 * ------------------------ Default
 * PBx  - input with pull-up
 */
#define VAL_GPIO_ODR            0xFFFFFFFF
#define VAL_GPIO_CRL            0x88888838      /*  PB7...PB0 */
#define VAL_GPIO_CRH            0x88888838      /* PB15...PB8 */

#define GPIO_USB_BASE   GPIOB_BASE
#define GPIO_LED_BASE   GPIOB_BASE

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST)

/* NeuG settings for ADC2 is default (PA0: Analog IN0, PA1: Analog IN1).  */
