/*
 * clk_gpio_init-stm32.c - Clock and GPIO initialization for STM32.
 *
 * Copyright (C) 2015  Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * receipents of GNU GPL by a written offer.
 *
 */

#define STM32_SW_HSI		(0 << 0)
#define STM32_SW_PLL		(2 << 0)
#define STM32_PLLSRC_HSI	(0 << 16)
#define STM32_PLLSRC_HSE	(1 << 16)

#define STM32_PLLXTPRE_DIV1	(0 << 17)
#define STM32_PLLXTPRE_DIV2	(1 << 17)

#define STM32_HPRE_DIV1		(0 << 4)

#define STM32_PPRE1_DIV1	(0 << 8)
#define STM32_PPRE1_DIV2	(4 << 8)

#define STM32_PPRE2_DIV1        (0 << 11)
#define STM32_PPRE2_DIV2	(4 << 11)

#define STM32_ADCPRE_DIV4	(1 << 14)
#define STM32_ADCPRE_DIV6       (2 << 14)

#define STM32_USBPRE_DIV1P5     (0 << 22)

#define STM32_MCO_NOCLOCK	(0 << 24)

#if defined(MCU_STM32F0)
#define STM32_PPRE1		STM32_PPRE1_DIV1
#define STM32_PLLSRC		STM32_PLLSRC_HSI
#define STM32_FLASHBITS		0x00000011
#define STM32_PLLCLKIN		(STM32_HSICLK / 2)
#else
#define STM32_PPRE1		STM32_PPRE1_DIV2
#define STM32_PLLSRC		STM32_PLLSRC_HSE
#define STM32_FLASHBITS		0x00000012
#define STM32_PLLCLKIN		(STM32_HSECLK / 1)
#endif

#define STM32_SW		STM32_SW_PLL
#define STM32_HPRE		STM32_HPRE_DIV1
#define STM32_PPRE2		STM32_PPRE2_DIV1
#define STM32_ADCPRE		STM32_ADCPRE_DIV6
#define STM32_MCOSEL		STM32_MCO_NOCLOCK
#define STM32_USBPRE            STM32_USBPRE_DIV1P5

#define STM32_PLLMUL		((STM32_PLLMUL_VALUE - 2) << 18)
#define STM32_PLLCLKOUT		(STM32_PLLCLKIN * STM32_PLLMUL_VALUE)
#define STM32_SYSCLK		STM32_PLLCLKOUT
#define STM32_HCLK		(STM32_SYSCLK / 1)


#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE	(PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x08000000)

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
#if defined(MCU_STM32F0)
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR2;
  volatile uint32_t CFGR3;
  volatile uint32_t CR2;
#endif
};

#define RCC_BASE		(AHBPERIPH_BASE + 0x1000)
static struct RCC *const RCC = (struct RCC *)RCC_BASE;

#define RCC_APB1ENR_USBEN	0x00800000
#define RCC_APB1RSTR_USBRST	0x00800000

#define RCC_CR_HSION		0x00000001
#define RCC_CR_HSIRDY		0x00000002
#define RCC_CR_HSITRIM		0x000000F8
#define RCC_CR_HSEON		0x00010000
#define RCC_CR_HSERDY		0x00020000
#define RCC_CR_PLLON		0x01000000
#define RCC_CR_PLLRDY		0x02000000

#define RCC_CFGR_SWS		0x0000000C
#define RCC_CFGR_SWS_HSI	0x00000000

#define RCC_AHBENR_CRCEN        0x0040

#if defined(MCU_STM32F0)
#define RCC_AHBRSTR_IOPARST	0x00020000
#define RCC_AHBRSTR_IOPBRST	0x00040000
#define RCC_AHBRSTR_IOPCRST	0x00080000
#define RCC_AHBRSTR_IOPDRST	0x00100000
#define RCC_AHBRSTR_IOPFRST	0x00400000

#define RCC_AHBENR_IOPAEN	0x00020000
#define RCC_AHBENR_IOPBEN	0x00040000
#define RCC_AHBENR_IOPCEN	0x00080000
#define RCC_AHBENR_IOPDEN	0x00100000
#define RCC_AHBENR_IOPFEN	0x00400000

#define RCC_APB2RSTR_SYSCFGRST	0x00000001
#define RCC_APB2ENR_SYSCFGEN	0x00000001
#else
#define RCC_APB2RSTR_AFIORST	0x00000001
#define RCC_APB2RSTR_IOPARST	0x00000004
#define RCC_APB2RSTR_IOPBRST	0x00000008
#define RCC_APB2RSTR_IOPCRST	0x00000010
#define RCC_APB2RSTR_IOPDRST	0x00000020
#define RCC_APB2RSTR_IOPERST	0x00000040
#define RCC_APB2RSTR_IOPFRST	0x00000080
#define RCC_APB2RSTR_IOPGRST	0x00000100

#define RCC_APB2ENR_AFIOEN	0x00000001
#define RCC_APB2ENR_IOPAEN	0x00000004
#define RCC_APB2ENR_IOPBEN	0x00000008
#define RCC_APB2ENR_IOPCEN	0x00000010
#define RCC_APB2ENR_IOPDEN	0x00000020
#define RCC_APB2ENR_IOPEEN	0x00000040
#define RCC_APB2ENR_IOPFEN	0x00000080
#define RCC_APB2ENR_IOPGEN	0x00000100
#endif

#if defined(MCU_STM32F0)
struct SYSCFG {
  volatile uint32_t CFGR1;
  uint32_t dummy0;
  volatile uint32_t EXTICR[4];
  volatile uint32_t CFGR2;
};
#define SYSCFG_CFGR1_MEM_MODE 0x03

#define SYSCFG_BASE	(APBPERIPH_BASE + 0x00010000)
static struct SYSCFG *const SYSCFG = (struct SYSCFG *)SYSCFG_BASE;
#endif

struct FLASH {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
};

#define FLASH_R_BASE	(AHBPERIPH_BASE + 0x2000)
static struct FLASH *const FLASH = (struct FLASH *)FLASH_R_BASE;

static void __attribute__((used))
clock_init (void)
{
  /* HSI setup */
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;
  /* Reset HSEON, HSEBYP, CSSON, and PLLON, not touching RCC_CR_HSITRIM */
  RCC->CR &= (RCC_CR_HSITRIM | RCC_CR_HSION);
  RCC->CFGR = 0;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;

#if !defined(MCU_STM32F0)
  /* HSE setup */
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;
#endif

  /* PLL setup */
  RCC->CFGR |= STM32_PLLMUL | STM32_PLLXTPRE | STM32_PLLSRC;
  RCC->CR   |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  /* Clock settings */
  RCC->CFGR = STM32_MCOSEL | STM32_USBPRE | STM32_PLLMUL | STM32_PLLXTPRE
    | STM32_PLLSRC | STM32_ADCPRE | STM32_PPRE2 | STM32_PPRE1 | STM32_HPRE;

  /*
   * We don't touch RCC->CR2, RCC->CFGR2, RCC->CFGR3, and RCC->CIR.
   */

  /* Flash setup */
  FLASH->ACR = STM32_FLASHBITS;

  /* CRC */
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  /* Switching on the configured clock source. */
  RCC->CFGR |= STM32_SW;
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;

#if defined(MCU_STM32F0)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB2RSTR = RCC_APB2RSTR_SYSCFGRST;
  RCC->APB2RSTR = 0;

# if defined(STM32F0_USE_VECTOR_ON_RAM)
  /* Use vectors on RAM */
  SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | 3;
# endif
#endif
}


#if defined(MCU_STM32F0)
struct GPIO {
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  uint16_t dummy0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  uint16_t dummy1;
  volatile uint16_t ODR;
  uint16_t dummy2;
  volatile uint16_t BSRR;
  uint16_t dummy3;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint16_t BRR;
  uint16_t dummy4;
};

#define GPIOA_BASE	(AHB2PERIPH_BASE + 0x0000)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE	(AHB2PERIPH_BASE + 0x0400)
#define GPIOB		((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE	(AHB2PERIPH_BASE + 0x0800)
#define GPIOC		((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE	(AHB2PERIPH_BASE + 0x0C00)
#define GPIOD		((struct GPIO *) GPIOD_BASE)
#define GPIOF_BASE	(AHB2PERIPH_BASE + 0x1400)
#define GPIOF		((struct GPIO *) GPIOF_BASE)
#else
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

#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP 0x00000800
#define AFIO_MAPR_SWJ_CFG_DISABLE         0x04000000
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE     0x02000000


struct GPIO {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
};

#define GPIOA_BASE	(APB2PERIPH_BASE + 0x0800)
#define GPIOA		((struct GPIO *) GPIOA_BASE)
#define GPIOB_BASE	(APB2PERIPH_BASE + 0x0C00)
#define GPIOB		((struct GPIO *) GPIOB_BASE)
#define GPIOC_BASE	(APB2PERIPH_BASE + 0x1000)
#define GPIOC		((struct GPIO *) GPIOC_BASE)
#define GPIOD_BASE	(APB2PERIPH_BASE + 0x1400)
#define GPIOD		((struct GPIO *) GPIOD_BASE)
#define GPIOE_BASE	(APB2PERIPH_BASE + 0x1800)
#define GPIOE		((struct GPIO *) GPIOE_BASE)
#endif

static struct GPIO *const GPIO_LED = (struct GPIO *)GPIO_LED_BASE;
#ifdef GPIO_USB_BASE
static struct GPIO *const GPIO_USB = (struct GPIO *)GPIO_USB_BASE;
#endif
#ifdef GPIO_OTHER_BASE
static struct GPIO *const GPIO_OTHER = (struct GPIO *)GPIO_OTHER_BASE;
#endif

static void __attribute__((used))
gpio_init (void)
{
  /* Enable GPIO clock. */
#if defined(MCU_STM32F0)
  RCC->AHBENR |= RCC_ENR_IOP_EN;
  RCC->AHBRSTR = RCC_RSTR_IOP_RST;
  RCC->AHBRSTR = 0;
#else
  RCC->APB2ENR |= RCC_ENR_IOP_EN;
  RCC->APB2RSTR = RCC_RSTR_IOP_RST;
  RCC->APB2RSTR = 0;
#endif

#if defined(MCU_STM32F0)
  GPIO_LED->OSPEEDR = VAL_GPIO_LED_OSPEEDR;
  GPIO_LED->OTYPER  = VAL_GPIO_LED_OTYPER;
  GPIO_LED->MODER   = VAL_GPIO_LED_MODER;
  GPIO_LED->PUPDR   = VAL_GPIO_LED_PUPDR;

#ifdef GPIO_OTHER_BASE
  GPIO_OTHER->OSPEEDR = VAL_GPIO_OTHER_OSPEEDR;
  GPIO_OTHER->OTYPER  = VAL_GPIO_OTHER_OTYPER;
  GPIO_OTHER->MODER   = VAL_GPIO_OTHER_MODER;
  GPIO_OTHER->PUPDR   = VAL_GPIO_OTHER_PUPDR;
#endif
#else
#ifdef AFIO_MAPR_SOMETHING
  AFIO->MAPR |= AFIO_MAPR_SOMETHING;
#endif

  /* LED is mandatory.  If it's on an independent port, we configure it.  */
  GPIO_LED->ODR = VAL_GPIO_LED_ODR;
  GPIO_LED->CRH = VAL_GPIO_LED_CRH;
  GPIO_LED->CRL = VAL_GPIO_LED_CRL;

  /* If there is USB enabler pin and it's independent, we configure it.  */
#if defined(GPIO_USB_BASE) && GPIO_USB_BASE != GPIO_LED_BASE
  GPIO_USB->ODR = VAL_GPIO_USB_ODR;
  GPIO_USB->CRH = VAL_GPIO_USB_CRH;
  GPIO_USB->CRL = VAL_GPIO_USB_CRL;
#endif

#ifdef GPIO_OTHER_BASE
  GPIO_OTHER->ODR = VAL_GPIO_OTHER_ODR;
  GPIO_OTHER->CRH = VAL_GPIO_OTHER_CRH;
  GPIO_OTHER->CRL = VAL_GPIO_OTHER_CRL;
#endif
#endif
}
