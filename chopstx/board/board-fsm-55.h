#define BOARD_NAME "FSM-55"
#define BOARD_ID    0x83433c76

/*
 * Running at 48MHz with HSI as clock source.
 * 
 */
#define MCU_STM32F0 1
/* __ARM_ARCH_6M__ */


#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              12
#define STM32_HSICLK                    8000000

#define GPIO_LED_BASE           GPIOA_BASE
#define GPIO_LED_SET_TO_EMIT    5
#define GPIO_OTHER_BASE         GPIOF_BASE /* USER BUTTON */

/*
 * Port A setup.
 * PA5  - ON (LED 1:ON 0:OFF)
 * PA4  - Pull DOWN
 */
#define VAL_GPIO_LED_MODER   0x00145555 /* Output Pin0-7, Pin9 and Pin10 */
#define VAL_GPIO_LED_OTYPER  0x0000001f /* Open-drain for Pin0-4, Push-Pull*/
#define VAL_GPIO_LED_OSPEEDR 0x003cffff /* High speed */
#define VAL_GPIO_LED_PUPDR   0x00000000 /* No pull-up/pull-down */


#define RCC_ENR_IOP_EN   (RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPFEN)
#define RCC_RSTR_IOP_RST (RCC_AHBRSTR_IOPARST | RCC_AHBRSTR_IOPFRST)

/*
 * Port F setup.
 * PF0  - USER Button
 * PF1  - SPEAKER
 */
#define VAL_GPIO_OTHER_MODER   0x00000004 /* Input Pin0, Output Pin1 */
#define VAL_GPIO_OTHER_OTYPER  0x00000000 /* Push-Pull Pin1 */
#define VAL_GPIO_OTHER_OSPEEDR 0x00000000
#define VAL_GPIO_OTHER_PUPDR   0x00000009 /* Pull-up Pin0, Pull-down Pin1 */
