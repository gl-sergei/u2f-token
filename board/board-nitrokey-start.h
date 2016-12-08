#define BOARD_NAME "NITROKEY-START"
#define BOARD_ID    0xad1e7ebd

#define STM32F10X_MD		/* Medium-density device */

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              6
#define STM32_HSECLK                    12000000

#define GPIO_LED_BASE   GPIOB_BASE
#define GPIO_LED_SET_TO_EMIT            0
#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_USB_SET_TO_ENABLE          15
#undef  GPIO_OTHER_BASE

/*
 * Port A setup.
 * PA0 - input with pull-up: AN0 for NeuG
 * PA1 - input with pull-up: AN1 for NeuG
 * PA2 - floating input
 * PA3 - floating input
 * PA4 - floating input
 * PA5 - floating input
 * PA6 - floating input
 * PA7 - Push pull output   (Red LED1 1:ON 0:OFF)
 * PA8 - floating input (smartcard, SCDSA)
 * PA9 - floating input
 * PA10 - floating input
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * PA15 - Push pull output  (USB_EN 1:ON 0:OFF)
 * ------------------------ Default
 * PA8  - input with pull-up.
 * PA9  - floating input.
 * PA10 - floating input.
 * PA13 - input with pull-up.
 * PA14 - input with pull-up.
 * PA15 - Push pull output   (USB 1:ON 0:OFF)
 */
#define VAL_GPIO_USB_ODR            0xFFFFE77F
#define VAL_GPIO_USB_CRL            0x34444488      /*  PA7...PA0 */
#define VAL_GPIO_USB_CRH            0x38811444      /* PA15...PA8 */

/*
 * Port B setup.
 * PB0  - Push pull output   (Green LED2 1:ON 0:OFF)
 * ------------------------ Default
 * PBx  - input with pull-up.
 */
#define VAL_GPIO_LED_ODR            0xFFFFFFFF
#define VAL_GPIO_LED_CRL            0x88888883      /*  PA7...PA0 */
#define VAL_GPIO_LED_CRH            0x88888888      /* PA15...PA8 */

#define RCC_ENR_IOP_EN      \
	(RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN)
#define RCC_RSTR_IOP_RST    \
	(RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_AFIORST)

#define AFIO_MAPR_SOMETHING   AFIO_MAPR_SWJ_CFG_JTAGDISABLE
