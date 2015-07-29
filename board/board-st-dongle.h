#define BOARD_NAME "ST Dongle"
/* echo -n "ST Dongle" | shasum -a 256 | sed -e 's/^.*\(........\)  -$/\1/' */
#define BOARD_ID    0x2cd4e471

#define STM32F10X_MD		/* Medium-density device */

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_LED_BASE   GPIOA_BASE
#define GPIO_LED_SET_TO_EMIT            9
#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_USB_SET_TO_ENABLE          15
#undef  GPIO_OTHER_BASE

/*
 * Port A setup.
 * PA0  - input with pull-up.  AN0
 * PA1  - input with pull-up.  AN1
 * PA9  - Push pull output 50MHz (LED 1:ON 0:OFF)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM) 
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * PA15 - Push pull output 50MHz (USB 1:ON 0:OFF)
 * ------------------------ Default
 * PAx  - input with pull-up
 */
#define VAL_GPIO_LED_ODR            0xFFFFE7FF
#define VAL_GPIO_LED_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_LED_CRH            0x38811838      /* PA15...PA8 */

#define RCC_ENR_IOP_EN      RCC_APB2ENR_IOPAEN
#define RCC_RSTR_IOP_RST    RCC_APB2RSTR_IOPARST
