#define BOARD_NAME "Maple Mini"
#define BOARD_ID    0x7a445272

#define STM32F10X_MD		/* Medium-density device */

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_LED_BASE   GPIOB_BASE
#define GPIO_LED_SET_TO_EMIT            1
#define GPIO_USB_BASE   GPIOB_BASE
#define GPIO_USB_CLEAR_TO_ENABLE        9
#define GPIO_OTHER_BASE GPIOA_BASE

/*
 * Port A setup.
 * PA0  - input with pull-up.  AN0
 * PA1  - input with pull-up.  AN1
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM) 
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up
 */
#define VAL_GPIO_OTHER_ODR            0xFFFFE7FF
#define VAL_GPIO_OTHER_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_OTHER_CRH            0x88811888      /* PA15...PA8 */

/*
 * Port B setup.
 * PB1  - Push pull output 50MHz (LED 1:ON 0:OFF)
 * PB9  - Push pull output 50MHz (USB 1:ON 0:OFF)
 * ------------------------ Default
 * PBx  - input with pull-up
 */
#define VAL_GPIO_LED_ODR            0xFFFFFFFF
#define VAL_GPIO_LED_CRL            0x88888838      /*  PB7...PB0 */
#define VAL_GPIO_LED_CRH            0x88888838      /* PB15...PB8 */

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST)
