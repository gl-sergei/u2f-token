#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_USB_CLEAR_TO_ENABLE        11
#define GPIO_LED_CLEAR_TO_EMIT          12

/*
 * Port C setup.
 * PC0  - input with pull-up.  AN10 for NeuG
 * PC1  - input with pull-up.  AN11 for NeuG
 * PC6  - input without pull-up/down
 * PC7  - input without pull-up/down
 * PC11 - Open-drain output 50MHz (USB disconnect).
 * PC12 - Push Pull output 50MHz (LED).
 * ------------------------ Default
 * PCx  - input with pull-up
 */
#define VAL_GPIO_ODR            0xFFFFFFFF
#define VAL_GPIO_CRL            0x44888888      /*  PC7...PC0 */
#define VAL_GPIO_CRH            0x88837888      /* PC15...PC8 */

#define GPIO_USB_BASE   GPIOC_BASE
#define GPIO_LED_BASE   GPIOC_BASE

#define RCC_ENR_IOP_EN      RCC_APB2ENR_IOPCEN
#define RCC_RSTR_IOP_RST    RCC_APB2RSTR_IOPCRST

/* NeuG settings for ADC2.  */
#define NEUG_ADC_SETTING2_SMPR1 ADC_SMPR1_SMP_AN10(ADC_SAMPLE_1P5) \
                              | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_1P5)
#define NEUG_ADC_SETTING2_SMPR2 0
#define NEUG_ADC_SETTING2_SQR3  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)   \
                              | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11)
#define NEUG_ADC_SETTING2_NUM_CHANNELS 2
