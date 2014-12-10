#define FLASH_PAGE_SIZE 1024

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#undef  GPIO_USB_CLEAR_TO_ENABLE
#define GPIO_LED_SET_TO_EMIT            8

/* For pin-cir settings of Gnuk */
#define TIMx                  TIM3
#define INTR_REQ_TIM          TIM3_IRQ
#define AFIO_EXTICR_INDEX     1
#define AFIO_EXTICR1_EXTIx_Py AFIO_EXTICR2_EXTI5_PB
#define EXTI_PR               EXTI_PR_PR5
#define EXTI_IMR              EXTI_IMR_MR5
#define EXTI_FTSR_TR          EXTI_FTSR_TR5
#define INTR_REQ_EXTI         EXTI9_5_IRQ
#define ENABLE_RCC_APB1
#define RCC_APBnENR_TIMxEN    RCC_APB1ENR_TIM3EN
#define RCC_APBnRSTR_TIMxRST  RCC_APB1RSTR_TIM3RST
#define AFIO_MAPR_SOMETHING   AFIO_MAPR_TIM3_REMAP_PARTIALREMAP
                              /* Remap (PB4, PB5) -> (TIM3_CH1, TIM3_CH2) */

/*
 * Port A setup.
 * PA0  - input with pull-up.  AN0
 * PA1  - input with pull-up.  AN1
 * PA8  - Push pull output 10MHz (LED 1:ON 0:OFF)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM) 
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up
 */
#define VAL_GPIO_ODR            0xFFFFE7FF
#define VAL_GPIO_CRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIO_CRH            0x88811881      /* PA15...PA8 */

#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_LED_BASE   GPIOA_BASE

#define RCC_ENR_IOP_EN   \
  (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN)
#define RCC_RSTR_IOP_RST \
  (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_AFIORST)

/* NeuG settings for ADC2 is default (PA0: Analog IN0, PA1: Analog IN1).  */

#define GPIO_OTHER_BASE GPIOB_BASE
/*
 * Port B setup.
 * PB4  - (TIM3_CH1) input with pull-up
 * PB5  - (TIM3_CH2) input with pull-up, connected to CIR module
 * Everything input with pull-up except:
 * PB0  - (TIM3_CH3) input with pull-down
 */
#define VAL_GPIO_OTHER_ODR 0xFFFFFFFE
#define VAL_GPIO_OTHER_CRL 0x88888888      /*  PB7...PB0 */
#define VAL_GPIO_OTHER_CRH 0x88888888      /* PB15...PB8 */
