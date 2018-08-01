#define PERIPH_BASE	0x40000000
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)

struct RCC {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
};

#define RCC_BASE		(AHBPERIPH_BASE + 0x1000)
static struct RCC *const RCC = (struct RCC *)RCC_BASE;

#define RCC_AHBENR_DMA1EN       0x00000001
#define RCC_AHBENR_CRCEN        0x00000040

#define RCC_APB2ENR_ADC1EN      0x00000200
#define RCC_APB2ENR_ADC2EN      0x00000400
#define RCC_APB2ENR_TIM1EN      0x00000800
#define RCC_APB1ENR_TIM2EN      0x00000001
#define RCC_APB1ENR_TIM3EN      0x00000002
#define RCC_APB1ENR_TIM4EN      0x00000004

#define RCC_APB2RSTR_ADC1RST    0x00000200
#define RCC_APB2RSTR_ADC2RST    0x00000400
#define RCC_APB2RSTR_TIM1RST    0x00000800
#define RCC_APB1RSTR_TIM2RST    0x00000001
#define RCC_APB1RSTR_TIM3RST    0x00000002
#define RCC_APB1RSTR_TIM4RST    0x00000004

#define  CRC_CR_RESET                        0x00000001

struct CRC {
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
};

#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)
static struct CRC *const CRC = (struct CRC *)CRC_BASE;


struct ADC {
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
};

#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)

static struct ADC *const ADC1 = (struct ADC *)ADC1_BASE;
static struct ADC *const ADC2 = (struct ADC *)ADC2_BASE;

#define  ADC_CR1_DUALMOD_0       0x00010000
#define  ADC_CR1_DUALMOD_1       0x00020000
#define  ADC_CR1_DUALMOD_2       0x00040000
#define  ADC_CR1_DUALMOD_3       0x00080000

#define  ADC_CR1_SCAN            0x00000100

#define  ADC_CR2_ADON            0x00000001
#define  ADC_CR2_CONT            0x00000002
#define  ADC_CR2_CAL             0x00000004
#define  ADC_CR2_RSTCAL          0x00000008
#define  ADC_CR2_DMA             0x00000100
#define  ADC_CR2_ALIGN           0x00000800
#define  ADC_CR2_EXTSEL          0x000E0000
#define  ADC_CR2_EXTSEL_0        0x00020000
#define  ADC_CR2_EXTSEL_1        0x00040000
#define  ADC_CR2_EXTSEL_2        0x00080000
#define  ADC_CR2_EXTTRIG         0x00100000
#define  ADC_CR2_SWSTART         0x00400000
#define  ADC_CR2_TSVREFE         0x00800000

struct DMA_Channel {
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
};

struct DMA {
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
};

#define STM32_DMA_CR_MINC           DMA_CCR1_MINC
#define STM32_DMA_CR_MSIZE_WORD     DMA_CCR1_MSIZE_1
#define STM32_DMA_CR_PSIZE_WORD     DMA_CCR1_PSIZE_1
#define STM32_DMA_CR_TCIE           DMA_CCR1_TCIE
#define STM32_DMA_CR_TEIE           DMA_CCR1_TEIE
#define STM32_DMA_CR_HTIE           DMA_CCR1_HTIE
#define STM32_DMA_ISR_TEIF          DMA_ISR_TEIF1
#define STM32_DMA_ISR_HTIF          DMA_ISR_HTIF1
#define STM32_DMA_ISR_TCIF          DMA_ISR_TCIF1

#define STM32_DMA_ISR_MASK          0x0F
#define STM32_DMA_CCR_RESET_VALUE   0x00000000
#define STM32_DMA_CR_PL_MASK        DMA_CCR1_PL
#define STM32_DMA_CR_PL(n)          ((n) << 12)

#define  DMA_CCR1_EN                         0x00000001
#define  DMA_CCR1_TCIE                       0x00000002
#define  DMA_CCR1_HTIE                       0x00000004
#define  DMA_CCR1_TEIE                       0x00000008
#define  DMA_CCR1_DIR                        0x00000010
#define  DMA_CCR1_CIRC                       0x00000020
#define  DMA_CCR1_PINC                       0x00000040
#define  DMA_CCR1_MINC                       0x00000080
#define  DMA_CCR1_PSIZE                      0x00000300
#define  DMA_CCR1_PSIZE_0                    0x00000100
#define  DMA_CCR1_PSIZE_1                    0x00000200
#define  DMA_CCR1_MSIZE                      0x00000C00
#define  DMA_CCR1_MSIZE_0                    0x00000400
#define  DMA_CCR1_MSIZE_1                    0x00000800
#define  DMA_CCR1_PL                         0x00003000
#define  DMA_CCR1_PL_0                       0x00001000
#define  DMA_CCR1_PL_1                       0x00002000
#define  DMA_CCR1_MEM2MEM                    0x00004000

#define  DMA_ISR_GIF1                        0x00000001
#define  DMA_ISR_TCIF1                       0x00000002
#define  DMA_ISR_HTIF1                       0x00000004
#define  DMA_ISR_TEIF1                       0x00000008
#define  DMA_ISR_GIF2                        0x00000010
#define  DMA_ISR_TCIF2                       0x00000020
#define  DMA_ISR_HTIF2                       0x00000040
#define  DMA_ISR_TEIF2                       0x00000080
#define  DMA_ISR_GIF3                        0x00000100
#define  DMA_ISR_TCIF3                       0x00000200
#define  DMA_ISR_HTIF3                       0x00000400
#define  DMA_ISR_TEIF3                       0x00000800
#define  DMA_ISR_GIF4                        0x00001000
#define  DMA_ISR_TCIF4                       0x00002000
#define  DMA_ISR_HTIF4                       0x00004000
#define  DMA_ISR_TEIF4                       0x00008000
#define  DMA_ISR_GIF5                        0x00010000
#define  DMA_ISR_TCIF5                       0x00020000
#define  DMA_ISR_HTIF5                       0x00040000
#define  DMA_ISR_TEIF5                       0x00080000
#define  DMA_ISR_GIF6                        0x00100000
#define  DMA_ISR_TCIF6                       0x00200000
#define  DMA_ISR_HTIF6                       0x00400000
#define  DMA_ISR_TEIF6                       0x00800000
#define  DMA_ISR_GIF7                        0x01000000
#define  DMA_ISR_TCIF7                       0x02000000
#define  DMA_ISR_HTIF7                       0x04000000
#define  DMA_ISR_TEIF7                       0x08000000

#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
static struct DMA *const DMA1 = (struct DMA *)DMA1_BASE;

#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
static struct DMA_Channel *const DMA1_Channel1 =
  (struct DMA_Channel *)DMA1_Channel1_BASE;

/* System Control Block */
struct SCB
{
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile uint32_t PFR[2];
  volatile uint32_t DFR;
  volatile uint32_t ADR;
  volatile uint32_t MMFR[4];
  volatile uint32_t ISAR[5];
  uint32_t RESERVED0[5];
  volatile uint32_t CPACR;
};

#define SCS_BASE 0xE000E000
#define SCB_BASE (SCS_BASE + 0x0D00)
static struct SCB *const SCB = (struct SCB *)SCB_BASE;

/* Timer */
struct TIM
{
  volatile uint16_t CR1;   uint16_t  RESERVED0;
  volatile uint16_t CR2;   uint16_t  RESERVED1;
  volatile uint16_t SMCR;  uint16_t  RESERVED2;
  volatile uint16_t DIER;  uint16_t  RESERVED3;
  volatile uint16_t SR;    uint16_t  RESERVED4;
  volatile uint16_t EGR;   uint16_t  RESERVED5;
  volatile uint16_t CCMR1; uint16_t  RESERVED6;
  volatile uint16_t CCMR2; uint16_t  RESERVED7;
  volatile uint16_t CCER;  uint16_t  RESERVED8;
  volatile uint16_t CNT;   uint16_t  RESERVED9;
  volatile uint16_t PSC;   uint16_t  RESERVED10;
  volatile uint16_t ARR;   uint16_t  RESERVED11;
  volatile uint16_t RCR;   uint16_t  RESERVED12;
  volatile uint16_t CCR1;  uint16_t  RESERVED13;
  volatile uint16_t CCR2;  uint16_t  RESERVED14;
  volatile uint16_t CCR3;  uint16_t  RESERVED15;
  volatile uint16_t CCR4;  uint16_t  RESERVED16;
  volatile uint16_t BDTR;  uint16_t  RESERVED17;
  volatile uint16_t DCR;   uint16_t  RESERVED18;
  volatile uint16_t DMAR;  uint16_t  RESERVED19;
};

#define TIM2_BASE 0x40000000
#define TIM3_BASE 0x40000400
#define TIM4_BASE 0x40000800
static struct TIM *const TIM2 = (struct TIM *)TIM2_BASE;
static struct TIM *const TIM3 = (struct TIM *)TIM3_BASE;
static struct TIM *const TIM4 = (struct TIM *)TIM4_BASE;

#define  TIM_CR1_CEN   	0x0001
#define  TIM_CR1_UDIS  	0x0002
#define  TIM_CR1_URS   	0x0004
#define  TIM_CR1_OPM   	0x0008
#define  TIM_CR1_DIR   	0x0010
#define  TIM_CR1_CMS   	0x0060
#define  TIM_CR1_CMS_0 	0x0020
#define  TIM_CR1_CMS_1 	0x0040
#define  TIM_CR1_ARPE  	0x0080
#define  TIM_CR1_CKD   	0x0300
#define  TIM_CR1_CKD_0 	0x0100
#define  TIM_CR1_CKD_1 	0x0200

#define  TIM_CR2_CCPC  	0x0001
#define  TIM_CR2_CCUS  	0x0004
#define  TIM_CR2_CCDS  	0x0008
#define  TIM_CR2_MMS   	0x0070
#define  TIM_CR2_MMS_0 	0x0010
#define  TIM_CR2_MMS_1 	0x0020
#define  TIM_CR2_MMS_2 	0x0040
#define  TIM_CR2_TI1S  	0x0080
#define  TIM_CR2_OIS1  	0x0100
#define  TIM_CR2_OIS1N 	0x0200
#define  TIM_CR2_OIS2  	0x0400
#define  TIM_CR2_OIS2N 	0x0800
#define  TIM_CR2_OIS3  	0x1000
#define  TIM_CR2_OIS3N 	0x2000
#define  TIM_CR2_OIS4  	0x4000

#define  TIM_SMCR_SMS   0x0007
#define  TIM_SMCR_SMS_0 0x0001
#define  TIM_SMCR_SMS_1 0x0002
#define  TIM_SMCR_SMS_2 0x0004
#define  TIM_SMCR_TS    0x0070
#define  TIM_SMCR_TS_0  0x0010
#define  TIM_SMCR_TS_1  0x0020
#define  TIM_SMCR_TS_2  0x0040
#define  TIM_SMCR_MSM   0x0080

#define  TIM_SMCR_ETF   0x0F00
#define  TIM_SMCR_ETF_0 0x0100
#define  TIM_SMCR_ETF_1 0x0200
#define  TIM_SMCR_ETF_2 0x0400
#define  TIM_SMCR_ETF_3 0x0800

#define  TIM_SMCR_ETPS   0x3000
#define  TIM_SMCR_ETPS_0 0x1000
#define  TIM_SMCR_ETPS_1 0x2000

#define  TIM_SMCR_ECE    0x4000
#define  TIM_SMCR_ETP    0x8000

#define  TIM_DIER_UIE    0x0001
#define  TIM_DIER_CC1IE  0x0002
#define  TIM_DIER_CC2IE  0x0004
#define  TIM_DIER_CC3IE  0x0008
#define  TIM_DIER_CC4IE  0x0010
#define  TIM_DIER_COMIE  0x0020
#define  TIM_DIER_TIE    0x0040
#define  TIM_DIER_BIE    0x0080
#define  TIM_DIER_UDE    0x0100
#define  TIM_DIER_CC1DE  0x0200
#define  TIM_DIER_CC2DE  0x0400
#define  TIM_DIER_CC3DE  0x0800
#define  TIM_DIER_CC4DE  0x1000
#define  TIM_DIER_COMDE  0x2000
#define  TIM_DIER_TDE    0x4000

#define  TIM_SR_UIF      0x0001
#define  TIM_SR_CC1IF    0x0002
#define  TIM_SR_CC2IF    0x0004
#define  TIM_SR_CC3IF    0x0008
#define  TIM_SR_CC4IF    0x0010
#define  TIM_SR_COMIF    0x0020
#define  TIM_SR_TIF      0x0040
#define  TIM_SR_BIF      0x0080
#define  TIM_SR_CC1OF    0x0200
#define  TIM_SR_CC2OF    0x0400
#define  TIM_SR_CC3OF    0x0800
#define  TIM_SR_CC4OF    0x1000

#define  TIM_EGR_UG      0x01
#define  TIM_EGR_CC1G    0x02
#define  TIM_EGR_CC2G    0x04
#define  TIM_EGR_CC3G    0x08
#define  TIM_EGR_CC4G    0x10
#define  TIM_EGR_COMG    0x20
#define  TIM_EGR_TG      0x40
#define  TIM_EGR_BG      0x80

#define  TIM_CCMR1_CC1S   0x0003
#define  TIM_CCMR1_CC1S_0 0x0001
#define  TIM_CCMR1_CC1S_1 0x0002

#define  TIM_CCMR1_OC1FE  0x0004
#define  TIM_CCMR1_OC1PE  0x0008

#define  TIM_CCMR1_OC1M   0x0070
#define  TIM_CCMR1_OC1M_0 0x0010
#define  TIM_CCMR1_OC1M_1 0x0020
#define  TIM_CCMR1_OC1M_2 0x0040

#define  TIM_CCMR1_OC1CE  0x0080

#define  TIM_CCMR1_CC2S   0x0300
#define  TIM_CCMR1_CC2S_0 0x0100
#define  TIM_CCMR1_CC2S_1 0x0200

#define  TIM_CCMR1_OC2FE  0x0400
#define  TIM_CCMR1_OC2PE  0x0800

#define  TIM_CCMR1_OC2M   0x7000
#define  TIM_CCMR1_OC2M_0 0x1000
#define  TIM_CCMR1_OC2M_1 0x2000
#define  TIM_CCMR1_OC2M_2 0x4000

#define  TIM_CCMR1_OC2CE  0x8000


#define  TIM_CCMR1_IC1PSC   0x000C
#define  TIM_CCMR1_IC1PSC_0 0x0004
#define  TIM_CCMR1_IC1PSC_1 0x0008

#define  TIM_CCMR1_IC1F     0x00F0
#define  TIM_CCMR1_IC1F_0   0x0010
#define  TIM_CCMR1_IC1F_1   0x0020
#define  TIM_CCMR1_IC1F_2   0x0040
#define  TIM_CCMR1_IC1F_3   0x0080

#define  TIM_CCMR1_IC2PSC   0x0C00
#define  TIM_CCMR1_IC2PSC_0 0x0400
#define  TIM_CCMR1_IC2PSC_1 0x0800

#define  TIM_CCMR1_IC2F     0xF000
#define  TIM_CCMR1_IC2F_0   0x1000
#define  TIM_CCMR1_IC2F_1   0x2000
#define  TIM_CCMR1_IC2F_2   0x4000
#define  TIM_CCMR1_IC2F_3   0x8000

#define  TIM_CCMR2_CC3S     0x0003
#define  TIM_CCMR2_CC3S_0   0x0001
#define  TIM_CCMR2_CC3S_1   0x0002

#define  TIM_CCMR2_OC3FE    0x0004
#define  TIM_CCMR2_OC3PE    0x0008

#define  TIM_CCMR2_OC3M     0x0070
#define  TIM_CCMR2_OC3M_0   0x0010
#define  TIM_CCMR2_OC3M_1   0x0020
#define  TIM_CCMR2_OC3M_2   0x0040

#define  TIM_CCMR2_OC3CE    0x0080

#define  TIM_CCMR2_CC4S     0x0300
#define  TIM_CCMR2_CC4S_0   0x0100
#define  TIM_CCMR2_CC4S_1   0x0200

#define  TIM_CCMR2_OC4FE    0x0400
#define  TIM_CCMR2_OC4PE    0x0800

#define  TIM_CCMR2_OC4M     0x7000
#define  TIM_CCMR2_OC4M_0   0x1000
#define  TIM_CCMR2_OC4M_1   0x2000
#define  TIM_CCMR2_OC4M_2   0x4000

#define  TIM_CCMR2_OC4CE    0x8000


#define  TIM_CCMR2_IC3PSC   0x000C
#define  TIM_CCMR2_IC3PSC_0 0x0004
#define  TIM_CCMR2_IC3PSC_1 0x0008

#define  TIM_CCMR2_IC3F     0x00F0
#define  TIM_CCMR2_IC3F_0   0x0010
#define  TIM_CCMR2_IC3F_1   0x0020
#define  TIM_CCMR2_IC3F_2   0x0040
#define  TIM_CCMR2_IC3F_3   0x0080

#define  TIM_CCMR2_IC4PSC   0x0C00
#define  TIM_CCMR2_IC4PSC_0 0x0400
#define  TIM_CCMR2_IC4PSC_1 0x0800

#define  TIM_CCMR2_IC4F     0xF000
#define  TIM_CCMR2_IC4F_0   0x1000
#define  TIM_CCMR2_IC4F_1   0x2000
#define  TIM_CCMR2_IC4F_2   0x4000
#define  TIM_CCMR2_IC4F_3   0x8000

#define  TIM_CCER_CC1E      0x0001
#define  TIM_CCER_CC1P      0x0002
#define  TIM_CCER_CC1NE     0x0004
#define  TIM_CCER_CC1NP     0x0008
#define  TIM_CCER_CC2E      0x0010
#define  TIM_CCER_CC2P      0x0020
#define  TIM_CCER_CC2NE     0x0040
#define  TIM_CCER_CC2NP     0x0080
#define  TIM_CCER_CC3E      0x0100
#define  TIM_CCER_CC3P      0x0200
#define  TIM_CCER_CC3NE     0x0400
#define  TIM_CCER_CC3NP     0x0800
#define  TIM_CCER_CC4E      0x1000
#define  TIM_CCER_CC4P      0x2000

#define  TIM_CNT_CNT        0xFFFF

#define  TIM_PSC_PSC        0xFFFF

#define  TIM_ARR_ARR        0xFFFF

#define  TIM_RCR_REP        0xFF

#define  TIM_CCR1_CCR1      0xFFFF
#define  TIM_CCR2_CCR2      0xFFFF
#define  TIM_CCR3_CCR3      0xFFFF
#define  TIM_CCR4_CCR4      0xFFFF

#define  TIM_BDTR_DTG       0x00FF
#define  TIM_BDTR_DTG_0     0x0001
#define  TIM_BDTR_DTG_1     0x0002
#define  TIM_BDTR_DTG_2     0x0004
#define  TIM_BDTR_DTG_3     0x0008
#define  TIM_BDTR_DTG_4     0x0010
#define  TIM_BDTR_DTG_5     0x0020
#define  TIM_BDTR_DTG_6     0x0040
#define  TIM_BDTR_DTG_7     0x0080

#define  TIM_BDTR_LOCK      0x0300
#define  TIM_BDTR_LOCK_0    0x0100
#define  TIM_BDTR_LOCK_1    0x0200

#define  TIM_BDTR_OSSI      0x0400
#define  TIM_BDTR_OSSR      0x0800
#define  TIM_BDTR_BKE       0x1000
#define  TIM_BDTR_BKP       0x2000
#define  TIM_BDTR_AOE       0x4000
#define  TIM_BDTR_MOE       0x8000

#define  TIM_DCR_DBA        0x001F
#define  TIM_DCR_DBA_0      0x0001
#define  TIM_DCR_DBA_1      0x0002
#define  TIM_DCR_DBA_2      0x0004
#define  TIM_DCR_DBA_3      0x0008
#define  TIM_DCR_DBA_4      0x0010

#define  TIM_DCR_DBL        0x1F00
#define  TIM_DCR_DBL_0      0x0100
#define  TIM_DCR_DBL_1      0x0200
#define  TIM_DCR_DBL_2      0x0400
#define  TIM_DCR_DBL_3      0x0800
#define  TIM_DCR_DBL_4      0x1000

#define  TIM_DMAR_DMAB      0xFFFF

struct EXTI
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
};

#define EXTI_BASE 0x40010400
static struct EXTI *const EXTI = (struct EXTI *)EXTI_BASE;

#define  EXTI_IMR_MR0       0x00000001
#define  EXTI_IMR_MR1       0x00000002
#define  EXTI_IMR_MR2       0x00000004
#define  EXTI_IMR_MR3       0x00000008
#define  EXTI_IMR_MR4       0x00000010
#define  EXTI_IMR_MR5       0x00000020
#define  EXTI_IMR_MR6       0x00000040
#define  EXTI_IMR_MR7       0x00000080
#define  EXTI_IMR_MR8       0x00000100
#define  EXTI_IMR_MR9       0x00000200
#define  EXTI_IMR_MR10      0x00000400
#define  EXTI_IMR_MR11      0x00000800
#define  EXTI_IMR_MR12      0x00001000
#define  EXTI_IMR_MR13      0x00002000
#define  EXTI_IMR_MR14      0x00004000
#define  EXTI_IMR_MR15      0x00008000
#define  EXTI_IMR_MR16      0x00010000
#define  EXTI_IMR_MR17      0x00020000
#define  EXTI_IMR_MR18      0x00040000
#define  EXTI_IMR_MR19      0x00080000

#define  EXTI_EMR_MR0       0x00000001
#define  EXTI_EMR_MR1       0x00000002
#define  EXTI_EMR_MR2       0x00000004
#define  EXTI_EMR_MR3       0x00000008
#define  EXTI_EMR_MR4       0x00000010
#define  EXTI_EMR_MR5       0x00000020
#define  EXTI_EMR_MR6       0x00000040
#define  EXTI_EMR_MR7       0x00000080
#define  EXTI_EMR_MR8       0x00000100
#define  EXTI_EMR_MR9       0x00000200
#define  EXTI_EMR_MR10      0x00000400
#define  EXTI_EMR_MR11      0x00000800
#define  EXTI_EMR_MR12      0x00001000
#define  EXTI_EMR_MR13      0x00002000
#define  EXTI_EMR_MR14      0x00004000
#define  EXTI_EMR_MR15      0x00008000
#define  EXTI_EMR_MR16      0x00010000
#define  EXTI_EMR_MR17      0x00020000
#define  EXTI_EMR_MR18      0x00040000
#define  EXTI_EMR_MR19      0x00080000

#define  EXTI_RTSR_TR0      0x00000001
#define  EXTI_RTSR_TR1      0x00000002
#define  EXTI_RTSR_TR2      0x00000004
#define  EXTI_RTSR_TR3      0x00000008
#define  EXTI_RTSR_TR4      0x00000010
#define  EXTI_RTSR_TR5      0x00000020
#define  EXTI_RTSR_TR6      0x00000040
#define  EXTI_RTSR_TR7      0x00000080
#define  EXTI_RTSR_TR8      0x00000100
#define  EXTI_RTSR_TR9      0x00000200
#define  EXTI_RTSR_TR10     0x00000400
#define  EXTI_RTSR_TR11     0x00000800
#define  EXTI_RTSR_TR12     0x00001000
#define  EXTI_RTSR_TR13     0x00002000
#define  EXTI_RTSR_TR14     0x00004000
#define  EXTI_RTSR_TR15     0x00008000
#define  EXTI_RTSR_TR16     0x00010000
#define  EXTI_RTSR_TR17     0x00020000
#define  EXTI_RTSR_TR18     0x00040000
#define  EXTI_RTSR_TR19     0x00080000

#define  EXTI_FTSR_TR0      0x00000001
#define  EXTI_FTSR_TR1      0x00000002
#define  EXTI_FTSR_TR2      0x00000004
#define  EXTI_FTSR_TR3      0x00000008
#define  EXTI_FTSR_TR4      0x00000010
#define  EXTI_FTSR_TR5      0x00000020
#define  EXTI_FTSR_TR6      0x00000040
#define  EXTI_FTSR_TR7      0x00000080
#define  EXTI_FTSR_TR8      0x00000100
#define  EXTI_FTSR_TR9      0x00000200
#define  EXTI_FTSR_TR10     0x00000400
#define  EXTI_FTSR_TR11     0x00000800
#define  EXTI_FTSR_TR12     0x00001000
#define  EXTI_FTSR_TR13     0x00002000
#define  EXTI_FTSR_TR14     0x00004000
#define  EXTI_FTSR_TR15     0x00008000
#define  EXTI_FTSR_TR16     0x00010000
#define  EXTI_FTSR_TR17     0x00020000
#define  EXTI_FTSR_TR18     0x00040000
#define  EXTI_FTSR_TR19     0x00080000

#define  EXTI_SWIER_SWIER0  0x00000001
#define  EXTI_SWIER_SWIER1  0x00000002
#define  EXTI_SWIER_SWIER2  0x00000004
#define  EXTI_SWIER_SWIER3  0x00000008
#define  EXTI_SWIER_SWIER4  0x00000010
#define  EXTI_SWIER_SWIER5  0x00000020
#define  EXTI_SWIER_SWIER6  0x00000040
#define  EXTI_SWIER_SWIER7  0x00000080
#define  EXTI_SWIER_SWIER8  0x00000100
#define  EXTI_SWIER_SWIER9  0x00000200
#define  EXTI_SWIER_SWIER10 0x00000400
#define  EXTI_SWIER_SWIER11 0x00000800
#define  EXTI_SWIER_SWIER12 0x00001000
#define  EXTI_SWIER_SWIER13 0x00002000
#define  EXTI_SWIER_SWIER14 0x00004000
#define  EXTI_SWIER_SWIER15 0x00008000
#define  EXTI_SWIER_SWIER16 0x00010000
#define  EXTI_SWIER_SWIER17 0x00020000
#define  EXTI_SWIER_SWIER18 0x00040000
#define  EXTI_SWIER_SWIER19 0x00080000

#define  EXTI_PR_PR0        0x00000001
#define  EXTI_PR_PR1        0x00000002
#define  EXTI_PR_PR2        0x00000004
#define  EXTI_PR_PR3        0x00000008
#define  EXTI_PR_PR4        0x00000010
#define  EXTI_PR_PR5        0x00000020
#define  EXTI_PR_PR6        0x00000040
#define  EXTI_PR_PR7        0x00000080
#define  EXTI_PR_PR8        0x00000100
#define  EXTI_PR_PR9        0x00000200
#define  EXTI_PR_PR10       0x00000400
#define  EXTI_PR_PR11       0x00000800
#define  EXTI_PR_PR12       0x00001000
#define  EXTI_PR_PR13       0x00002000
#define  EXTI_PR_PR14       0x00004000
#define  EXTI_PR_PR15       0x00008000
#define  EXTI_PR_PR16       0x00010000
#define  EXTI_PR_PR17       0x00020000
#define  EXTI_PR_PR18       0x00040000
#define  EXTI_PR_PR19       0x00080000

#define EXTI0_IRQ    6
#define EXTI1_IRQ    7
#define EXTI2_IRQ    8
#define EXTI9_5_IRQ 23
#define TIM2_IRQ    28
#define TIM3_IRQ    29
#define TIM4_IRQ    30

struct AFIO
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;
};

#define AFIO_BASE 0x40010000
static struct AFIO *const AFIO = (struct AFIO *)AFIO_BASE;

#define AFIO_EXTICR1_EXTI0_PA 0x0000
#define AFIO_EXTICR1_EXTI0_PB 0x0001
#define AFIO_EXTICR1_EXTI0_PC 0x0002
#define AFIO_EXTICR1_EXTI0_PD 0x0003

#define AFIO_EXTICR1_EXTI1_PA 0x0000
#define AFIO_EXTICR1_EXTI1_PB 0x0010
#define AFIO_EXTICR1_EXTI1_PC 0x0020
#define AFIO_EXTICR1_EXTI1_PD 0x0030

#define AFIO_EXTICR1_EXTI2_PA 0x0000
#define AFIO_EXTICR1_EXTI2_PB 0x0100
#define AFIO_EXTICR1_EXTI2_PC 0x0200
#define AFIO_EXTICR1_EXTI2_PD 0x0300

#define AFIO_EXTICR1_EXTI3_PA 0x0000
#define AFIO_EXTICR1_EXTI3_PB 0x1000
#define AFIO_EXTICR1_EXTI3_PC 0x2000
#define AFIO_EXTICR1_EXTI3_PD 0x3000

#define AFIO_EXTICR2_EXTI4_PA 0x0000
#define AFIO_EXTICR2_EXTI4_PB 0x0001
#define AFIO_EXTICR2_EXTI4_PC 0x0002
#define AFIO_EXTICR2_EXTI4_PD 0x0003

#define AFIO_EXTICR2_EXTI5_PA 0x0000
#define AFIO_EXTICR2_EXTI5_PB 0x0010
#define AFIO_EXTICR2_EXTI5_PC 0x0020
#define AFIO_EXTICR2_EXTI5_PD 0x0030

#define AFIO_EXTICR2_EXTI6_PA 0x0000
#define AFIO_EXTICR2_EXTI6_PB 0x0100
#define AFIO_EXTICR2_EXTI6_PC 0x0200
#define AFIO_EXTICR2_EXTI6_PD 0x0300

#define AFIO_EXTICR2_EXTI7_PA 0x0000
#define AFIO_EXTICR2_EXTI7_PB 0x1000
#define AFIO_EXTICR2_EXTI7_PC 0x2000
#define AFIO_EXTICR2_EXTI7_PD 0x3000

#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP 0x00000800
#define AFIO_MAPR_SWJ_CFG_DISABLE         0x04000000
