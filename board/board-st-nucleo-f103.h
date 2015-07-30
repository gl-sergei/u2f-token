#define BOARD_NAME "ST Nucleo F103"
#define BOARD_ID    0x9b87c16d

/*
 * Please add X3 and USB cable to ST Nucleo F103.
 *
 * Solder X3 XTAL of 8MHz (and put C33 and C34 of 22pF).  
 * Solder the bridges for R35 and R37, since it's 0 ohm. 
 *
 * (Optional) Remove SB54 and SB55.
 *
 * At CN10, connect USB cable
 *  Vbus RED   --> 10 NC   ----------> CN7 (6 E5V)
 *  D+   GREEN --> 12 PA11 ---[1K5]--> CN6 (4 3V3)
 *  D-   WHITE --> 14 PA12
 *  GND  BLACK --> 20 GND
 */

#define STM32F10X_MD		/* Medium-density device */

#define STM32_PLLXTPRE                  STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE              9
#define STM32_HSECLK                    8000000

#define GPIO_LED_BASE   GPIOA_BASE
#define GPIO_LED_SET_TO_EMIT            5
#undef  GPIO_USB_BASE		/* No external DISCONNECT/RENUM circuit.  */
#undef  GPIO_OTHER_BASE

/*
 * Port A setup.
 * PA0  - input with pull-up.  AN0
 * PA1  - input with pull-up.  AN1
 * PA5  - Push pull output 50MHz (LED 1:ON 0:OFF)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled) (USBDM) 
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up
 */
#define VAL_GPIO_LED_ODR            0xFFFFE7FF
#define VAL_GPIO_LED_CRL            0x88388888      /*  PA7...PA0 */
#define VAL_GPIO_LED_CRH            0x88811888      /* PA15...PA8 */

#define RCC_ENR_IOP_EN      RCC_APB2ENR_IOPAEN
#define RCC_RSTR_IOP_RST    RCC_APB2RSTR_IOPARST
