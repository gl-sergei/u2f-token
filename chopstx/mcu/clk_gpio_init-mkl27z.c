/*
 * clk_gpio_init-mkl27z.c - Clock and GPIO initialization for Kinetis L.
 *
 * Copyright (C) 2016  Flying Stone Technology
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

#include <mcu/mkl27z.h>

struct MCG {
  volatile uint8_t C1;   /* MCG Control Register 1             */
  volatile uint8_t C2;   /* MCG Control Register 2             */
  uint8_t reserved0[4];  /*                                    */
  volatile uint8_t S;    /* MCG Status Register                */
  uint8_t reserved1[1];  /*                                    */
  volatile uint8_t SC;   /* MCG Status and Control Register    */
  uint8_t reserved2[15]; /*                                    */
  volatile uint8_t MC;   /* MCG Miscellaneous Control Register */
};
static struct MCG *const MCG = (struct MCG *)0x40064000;

struct USB_CLK_RECOVER {
  volatile uint8_t CTRL;           /* USB Clock             */
  uint8_t rsvd38[3];               /*     recovery control  */
  volatile uint8_t IRC_EN;         /* IRC48M oscillator     */
  uint8_t rsvd39[3];               /*     enable register   */
  volatile uint8_t INT_EN;         /* Clock recovery        */
  uint8_t rsvd40[3];               /*     interrupt enable  */
  volatile uint8_t INT_STATUS;     /* Clock recovery        */
                                   /*     interrupt status  */
};
static struct USB_CLK_RECOVER *const USB_CLK_RECOVER =
  (struct USB_CLK_RECOVER *)0x40072140;

static void __attribute__((used))
clock_init (void)
{
  SIM->CLKDIV1 = (SIM->CLKDIV1 & 0xF0070000)
               | (1 << 16)	/* OUTDIV4 = 001: Divide-by-2 */
               ;

  MCG->MC = 0x80; /* HIRC Enable, LIRC_DIV2=000: Division factor=1 */
  MCG->C1 = 0x00; /* Select HIRC clock, LIRC disabled. */
  /* Make sure HIRC clock is selected.  */
  while ((MCG->S & 0x0c) != 0)
    ;

  /* TPMSRC=IRC48M, USBSRC=IRC48M, CLOKOUTSEL=LPO, RTC-clock */
  SIM->SOPT2 = 0x01040060;

  SIM->SCGC4 = (1 << 18);	/* Enable USB FS clock        */
  SIM->SCGC5 = (1 << 10);	/* Enable Port B clock        */
  SIM->SCGC6 = (1 << 25)|1;	/* Enable TPM1 clock          */
  SIM->COPC = 0;		/* COP disabled               */

  /* Crystal-less USB setup.  */
  USB_CLK_RECOVER->IRC_EN = 0x02;
  USB_CLK_RECOVER->CTRL = 0x80;
}


static void __attribute__((used))
gpio_init (void)
{
  PORTB->PCR0 = (1<<8) /* GPIO                  */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* pull up select= 0     */
              ;
  PORTB->PCR1 = (1<<8) /* GPIO                  */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* pull up select= 0     */
              ;

  GPIOB->PDDR = (1 << 1) | (1 << 0); /* PTB0, PTB1 : Output    */
  GPIOB->PSOR = (1 << 0);            /* PTB0: Set  : Light off */
  GPIOB->PCOR = (1 << 1);            /* PTB1: Clear: Output 0  */
}
