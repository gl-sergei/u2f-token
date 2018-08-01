#define BOARD_NAME "CQ STARM"
#define BOARD_ID    0xc5480875

#define STM32F10X_MD		/* Medium-density device */

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_LED_BASE   GPIOC_BASE
#define GPIO_LED_SET_TO_EMIT            6
#undef  GPIO_USB_BASE		/* No external DISCONNECT/RENUM circuit.  */
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
 * Port C setup.
 * PC0  - Push Pull output 50MHz.
 * PC1  - Push Pull output 50MHz.
 * Everything input with pull-up except:
 * PC4  - Normal input      (ADC_IN14 : VoutY of LIS344ALH).
 * PC5  - Normal input      (ADC_IN15 : VoutZ of LIS344ALH).
 * PC6  - Push Pull output (LED).
 * (PC9  - SDCard CD)
 * (PC12 - SDCard CS)
 * PC14 - Normal input (XTAL).
 * PC15 - Normal input (XTAL).
 */
#define VAL_GPIO_LED_CRL            0x83448833      /*  PC7...PC0 */
#define VAL_GPIO_LED_CRH            0x44888888      /* PC15...PC8 */
#define VAL_GPIO_LED_ODR            0xFFFFFFFF

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPCEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST|RCC_APB2RSTR_IOPCRST)
