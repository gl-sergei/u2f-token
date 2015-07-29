#define BOARD_NAME "STM32F0 Discovery"
#define BOARD_ID    0xde4b4bc1

/*
 * Running at 48MHz with HSI as clock source.
 * 
 */
#define MCU_STM32F0 1
/* __ARM_ARCH_6M__ */


#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              12
#define STM32_HSICLK                    8000000

#define GPIO_LED_BASE           GPIOC_BASE
#define GPIO_LED_SET_TO_EMIT    8
#define GPIO_OTHER_BASE         GPIOA_BASE /* USER BUTTON */

/*
 * Port C setup.
 * PC9  - LED3 (LED 1:ON 0:OFF)
 * PC8  - LED4 (LED 1:ON 0:OFF)
 */
#define VAL_GPIO_LED_MODER   0x00050000 /* Output Pin9 and Pin8 */
#define VAL_GPIO_LED_OTYPER  0x00000000 /* Push-Pull */
#define VAL_GPIO_LED_OSPEEDR 0x000f0000 /* High speed: Pin9 and Pin8 */
#define VAL_GPIO_LED_PUPDR   0x00000000 /* No pull-up/pull-down */


#if 0
#define RCC_ENR_IOP_EN   (RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPCEN)
#define RCC_RSTR_IOP_RST (RCC_AHBRSTR_IOPARST | RCC_AHBRSTR_IOPCRST)
#else
#define RCC_ENR_IOP_EN   RCC_AHBENR_IOPCEN
#define RCC_RSTR_IOP_RST RCC_AHBRSTR_IOPCRST
#endif

/* ??? NeuG settings for ADC2 is default (PA0: Analog IN0, PA1: Analog IN1).  */

/*
 * Port A setup.
 * PA0  - USER Button
 */
#define VAL_GPIO_OTHER_MODER   0x00000000 /* Input Pin0 */
#define VAL_GPIO_OTHER_OTYPER  0x00000000 /* Push-Pull */
#define VAL_GPIO_OTHER_OSPEEDR 0x00000000
#define VAL_GPIO_OTHER_PUPDR   0x00000000 /* No pull-up/pull-down */
