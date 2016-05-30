#define PERIPH_BASE	0x40000000
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)

#define RCC_APB2RSTR_ADC1RST 0x00000200
#define RCC_APB2RSTR_ADC2RST 0x00000400

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
static struct RCC *const RCC = ((struct RCC *const)RCC_BASE);

#define RCC_AHBENR_DMA1EN       0x00000001
#define RCC_AHBENR_CRCEN        0x00000040

#define RCC_APB2ENR_ADC1EN      0x00000200
#define RCC_APB2ENR_ADC2EN      0x00000400

#define  CRC_CR_RESET                        0x00000001

struct CRC {
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
};

#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)
static struct CRC *const CRC = ((struct CRC *const)CRC_BASE);


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

static struct ADC *const ADC1 = (struct ADC *const)ADC1_BASE;
static struct ADC *const ADC2 = (struct ADC *const)ADC2_BASE;

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
static struct DMA *const DMA1 = (struct DMA *const)DMA1_BASE;

#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
static struct DMA_Channel *const DMA1_Channel1 =
  (struct DMA_Channel *const)DMA1_Channel1_BASE;

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
static struct SCB *const SCB = (struct SCB *const)SCB_BASE;
