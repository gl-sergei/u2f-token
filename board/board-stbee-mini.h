#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              6
#define STM32_HSECLK                    12000000

#define GPIO_USB_SET_TO_ENABLE          14
#define GPIO_LED_CLEAR_TO_EMIT          13

#if defined(PINPAD_CIR_SUPPORT) || defined(PINPAD_DIAL_SUPPORT)
#define HAVE_7SEGLED	1
/*
 * Timer assignment for CIR
 */
#define TIMx                  TIM3
#define INTR_REQ_TIM          TIM3_IRQ
#define AFIO_EXTICR_INDEX     0
#endif

#if defined(PINPAD_CIR_SUPPORT)
#define AFIO_EXTICR1_EXTIx_Py AFIO_EXTICR1_EXTI0_PB
#define EXTI_PR               EXTI_PR_PR0
#define EXTI_IMR              EXTI_IMR_MR0
#define EXTI_FTSR_TR          EXTI_FTSR_TR0
#define INTR_REQ_EXTI         EXTI0_IRQ
#define RCC_APBnENR_TIMxEN    RCC_APB1ENR_TIM3EN
#define RCC_APBnRSTR_TIMxRST  RCC_APB1RSTR_TIM3RST
#elif defined(PINPAD_DIAL_SUPPORT)
#define AFIO_EXTICR1_EXTIx_Py AFIO_EXTICR1_EXTI2_PB
#define EXTI_PR               EXTI_PR_PR2
#define EXTI_IMR              EXTI_IMR_MR2
#define EXTI_FTSR_TR          EXTI_FTSR_TR2
#define INTR_REQ_EXTI         EXTI2_IRQ
#define RCC_APBnENR_TIMxEN    RCC_APB1ENR_TIM4EN
#define RCC_APBnRSTR_TIMxRST  RCC_APB1RSTR_TIM4RST
#endif
#define ENABLE_RCC_APB1

#if defined(PINPAD_CIR_SUPPORT) || defined(PINPAD_DIAL_SUPPORT)
/*
 * Port A setup.
 * PA1  - Digital input with PullUp.  AN1 for NeuG
 * PA2  - Digital input with PullUp.  AN2 for NeuG
 * PA6  - (TIM3_CH1) input with pull-up
 * PA7  - (TIM3_CH2) input with pull-down
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * PA13 - Open Drain output (LED1 0:ON 1:OFF)
 * PA14 - Push pull output  (USB ENABLE 0:DISABLE 1:ENABLE)
 * PA15 - Open Drain output (LED2 0:ON 1:OFF)
 */
#define VAL_GPIO_ODR            0xFFFFE77F
#define VAL_GPIO_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_CRH            0x63611888      /* PA15...PA8 */

/*
 * Port B setup.
 * PB0  - Push pull output   (LED 1:ON 0:OFF)
 * ------------------------ Default
 * PBx  - input with pull-up.
 */
#define VAL_GPIO_LED_ODR            0xFFFFFFFF
#define VAL_GPIO_LED_CRL            0x88888888      /*  PB7...PB0 */
#define VAL_GPIO_LED_CRH            0x66666666      /* PB15...PB8 */

/* Port B setup. */
#define GPIOB_CIR               0
#define GPIOB_BUTTON            2
#define GPIOB_ROT_A             6
#define GPIOB_ROT_B             7

#define GPIOB_7SEG_DP           15
#define GPIOB_7SEG_A            14
#define GPIOB_7SEG_B            13
#define GPIOB_7SEG_C            12
#define GPIOB_7SEG_D            11
#define GPIOB_7SEG_E            10
#define GPIOB_7SEG_F            9
#define GPIOB_7SEG_G            8

#define RCC_ENR_IOP_EN      \
        (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN)
#define RCC_RSTR_IOP_RST    \
        (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_AFIORST)
#else
/*
 * Port A setup.
 * PA1  - Digital input with PullUp.  AN1 for NeuG
 * PA2  - Digital input with PullUp.  AN2 for NeuG
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * PA13 - Open Drain output (LED1 0:ON 1:OFF)
 * PA14 - Push pull output  (USB ENABLE 0:DISABLE 1:ENABLE)
 * PA15 - Open Drain output (LED2 0:ON 1:OFF)
 */
#define VAL_GPIO_ODR            0xFFFFE7FF
#define VAL_GPIO_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_CRH            0x63611888      /* PA15...PA8 */

#define RCC_ENR_IOP_EN      (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN)
#define RCC_RSTR_IOP_RST    (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_AFIORST)
#endif


#define GPIO_USB_BASE           GPIOA_BASE
#define GPIO_LED_BASE           GPIOA_BASE
#define AFIO_MAPR_SOMETHING     AFIO_MAPR_SWJ_CFG_DISABLE

/* NeuG settings for ADC2.  */
#define NEUG_ADC_SETTING2_SMPR1 0
#define NEUG_ADC_SETTING2_SMPR2 ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5)    \
                              | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5)
#define NEUG_ADC_SETTING2_SQR3  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)      \
                              | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
#define NEUG_ADC_SETTING2_NUM_CHANNELS 2
