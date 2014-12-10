#define FLASH_PAGE_SIZE 2048

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              6
#define STM32_HSECLK                    12000000

#define GPIO_USB_CLEAR_TO_ENABLE        3
#define GPIO_LED_CLEAR_TO_EMIT          4

/*
 * Port A setup.
 * PA0  - Normal input.
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 */
#define VAL_GPIO_OTHER_ODR      0xFFFFE7FF
#define VAL_GPIO_OTHER_CRL      0x88888884      /*  PA7...PA0 */
#define VAL_GPIO_OTHER_CRH      0x88811888      /* PA15...PA8 */

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPDEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST|RCC_APB2RSTR_IOPDRST)

/*
 * Port D setup.
 * PD3  - Push pull output  (USB_DISC 1:USB-DISABLE 0:USB-ENABLE) 2MHz
 * PD4  - Open Drain output 2MHz (LED1).
 */
#define VAL_GPIO_ODR            0xFFFFFFFF
#define VAL_GPIO_CRL            0x88862888      /*  PD7...PD0 */
#define VAL_GPIO_CRH            0x88888888      /* PD15...PD8 */

#define GPIO_USB_BASE   GPIOD_BASE
#define GPIO_LED_BASE   GPIOD_BASE
#define GPIO_OTHER_BASE GPIOA_BASE

/* NeuG settings for ADC2.  */
#define NEUG_ADC_SETTING2_SMPR1 ADC_SMPR1_SMP_AN10(ADC_SAMPLE_1P5) \
                              | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_1P5)
#define NEUG_ADC_SETTING2_SMPR2 0
#define NEUG_ADC_SETTING2_SQR3  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)   \
                              | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11)
#define NEUG_ADC_SETTING2_NUM_CHANNELS 2
