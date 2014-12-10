#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              6
#define STM32_HSECLK                    12000000

#define GPIO_USB_SET_TO_ENABLE          10
#define GPIO_LED_SET_TO_EMIT            0

/* For pin-cir settings of Gnuk */
#define TIMx                  TIM2
#define INTR_REQ_TIM          TIM2_IRQ
#define AFIO_EXTICR_INDEX     0
#define AFIO_EXTICR1_EXTIx_Py AFIO_EXTICR1_EXTI2_PA
#define EXTI_PR               EXTI_PR_PR2
#define EXTI_IMR              EXTI_IMR_MR2
#define EXTI_FTSR_TR          EXTI_FTSR_TR2
#define INTR_REQ_EXTI         EXTI2_IRQ
#define ENABLE_RCC_APB1
#define RCC_APBnENR_TIMxEN    RCC_APB1ENR_TIM2EN
#define RCC_APBnRSTR_TIMxRST  RCC_APB1RSTR_TIM2RST

/*
 * Port A setup.
 * PA0  - input with pull-up (TIM2_CH1): AN0 for NeuG
 * PA1  - input with pull-down (TIM2_CH2)
 * PA2  - input with pull-up (TIM2_CH3) connected to CIR module
 * PA3  - input with pull-up: external pin available to user
 * PA4  - Push pull output           (SPI1_NSS)
 * PA5  - Alternate Push pull output (SPI1_SCK)
 * PA6  - Alternate Push pull output (SPI1_MISO)
 * PA7  - Alternate Push pull output (SPI1_MOSI)
 * PA10 - Push pull output   (USB 1:ON 0:OFF)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * ------------------------ Default
 * PA8  - input with pull-up.
 * PA9  - input with pull-up.
 * PA13 - input with pull-up.
 * PA14 - input with pull-up.
 * PA15 - input with pull-up.
 */
#define VAL_GPIO_ODR            0xFFFFE7FD
#define VAL_GPIO_CRL            0xBBB38888      /*  PA7...PA0 */
#define VAL_GPIO_CRH            0x88811388      /* PA15...PA8 */

/*
 * Port B setup.
 * PB0  - Push pull output   (LED 1:ON 0:OFF)
 * PB1  - input with pull-up: AN9 for NeuG
 * ------------------------ Default
 * PBx  - input with pull-up.
 */
#define VAL_GPIO_LED_ODR            0xFFFFFFFF
#define VAL_GPIO_LED_CRL            0x88888883      /*  PA7...PA0 */
#define VAL_GPIO_LED_CRH            0x88888888      /* PA15...PA8 */

#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_LED_BASE   GPIOB_BASE

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST)

/* NeuG settings for ADC2.  */
#define NEUG_ADC_SETTING2_SMPR1 0
#define NEUG_ADC_SETTING2_SMPR2 ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5)    \
                              | ADC_SMPR2_SMP_AN9(ADC_SAMPLE_1P5)
#define NEUG_ADC_SETTING2_SQR3  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)      \
                              | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)
#define NEUG_ADC_SETTING2_NUM_CHANNELS 2
