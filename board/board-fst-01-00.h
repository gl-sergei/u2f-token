#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_USB_SET_TO_ENABLE          10
#define GPIO_LED_SET_TO_EMIT            8

/*
 * Port A setup.
 * PA0  - input with pull-up.  AN0
 * PA1  - input with pull-up.  AN1
 * PA8  - Push pull output 50MHz (LED 1:ON 0:OFF)
 * PA10 - Push pull output 50MHz (USB 1:ON 0:OFF)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM) 
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up
 */
#define VAL_GPIO_ODR            0xFFFFE7FF
#define VAL_GPIO_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_CRH            0x88811383      /* PA15...PA8 */

#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_LED_BASE   GPIOA_BASE

#define RCC_ENR_IOP_EN      RCC_APB2ENR_IOPAEN
#define RCC_RSTR_IOP_RST    RCC_APB2RSTR_IOPARST

/* NeuG settings for ADC2 is default (PA0: Analog IN0, PA1: Analog IN1).  */
