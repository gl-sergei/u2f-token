#define STM32_PLLXTPRE			STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE		9
#define STM32_HSECLK			8000000

#define GPIO_USB_CLEAR_TO_ENABLE	11
#define GPIO_LED_CLEAR_TO_EMIT		12

/*
 * PC0  - Digital input with PullUp.  AN10 for NeuG
 * PC1  - Digital input with PullUp.  AN11 for NeuG
 * PC6  - Normal input because there is an external resistor.
 * PC7  - Normal input because there is an external resistor.
 * PC11 - Open Drain output (USB disconnect).
 * PC12 - Push Pull output (LED).
 */
#define VAL_GPIO_ODR            0xFFFFFFFF
#define VAL_GPIO_CRL            0x44888888      /*  PC7...PC0 */
#define VAL_GPIO_CRH            0x88837888      /* PC15...PC8 */

#define GPIO_USB_BASE	GPIOC_BASE
#define GPIO_LED_BASE	GPIOC_BASE

#define RCC_APB2ENR_IOP_EN	RCC_APB2ENR_IOPCEN
#define RCC_APB2RSTR_IOP_RST	RCC_APB2RSTR_IOPCRST
