/*
 * clk_gpio_init-kl.c - Clock and GPIO initialization for Kinetis L.
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

#include <mcu/kl_sim.h>

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
static struct MCG *const MCG = (struct MCG *const)0x40064000;

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
  (struct USB_CLK_RECOVER *const)0x40072140;

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

  SIM->SOPT2 = 0x00040060;	/* USBSRC=IRC48, CLOKOUTSEL=LPO, RTC-clock */

  SIM->SCGC4 = (1 << 18);	/* Enable USB FS clock         */
  SIM->SCGC5 = (1 << 10);	/* Enable Port B clock        */
  SIM->COPC = 0;		/* COP disabled               */

  /* Crystal-less USB setup.  */
  USB_CLK_RECOVER->IRC_EN = 0x02;
  USB_CLK_RECOVER->CTRL = 0x80;
}


struct PORT {
  volatile uint32_t PCR0;  volatile uint32_t PCR1;
  volatile uint32_t PCR2;  volatile uint32_t PCR3;
  volatile uint32_t PCR4;  volatile uint32_t PCR5;
  volatile uint32_t PCR6;  volatile uint32_t PCR7;
  volatile uint32_t PCR8;  volatile uint32_t PCR9;
  volatile uint32_t PCR10; volatile uint32_t PCR11;
  volatile uint32_t PCR12; volatile uint32_t PCR13;
  volatile uint32_t PCR14; volatile uint32_t PCR15;
  volatile uint32_t PCR16; volatile uint32_t PCR17;
  volatile uint32_t PCR18; volatile uint32_t PCR19;
  volatile uint32_t PCR20; volatile uint32_t PCR21;
  volatile uint32_t PCR22; volatile uint32_t PCR23;
  volatile uint32_t PCR24; volatile uint32_t PCR25;
  volatile uint32_t PCR26; volatile uint32_t PCR27;
  volatile uint32_t PCR28; volatile uint32_t PCR29;
  volatile uint32_t PCR30; volatile uint32_t PCR31;
  volatile uint32_t GPCLR; volatile uint32_t GPCHR;
  uint32_t reserved[6];
  volatile uint32_t ISFR;
};
static struct PORT *const PORTB = (struct PORT *const)0x4004A000;
static struct PORT *const PORTD = (struct PORT *const)0x4004C000;
static struct PORT *const PORTE = (struct PORT *const)0x4004D000;

struct GPIO {
  volatile uint32_t PDOR; /* Port Data Output Register    */
  volatile uint32_t PSOR; /* Port Set Output Register     */
  volatile uint32_t PCOR; /* Port Clear Output Register   */
  volatile uint32_t PTOR; /* Port Toggle Output Register  */
  volatile uint32_t PDIR; /* Port Data Input Register     */
  volatile uint32_t PDDR; /* Port Data Direction Register */
};
static struct GPIO *const GPIOB = (struct GPIO *const)0x400FF040;
static struct GPIO *const GPIOD = (struct GPIO *const)0x400FF0C0;
static struct GPIO *const GPIOE = (struct GPIO *const)0x400FF100;


static void __attribute__((used))
gpio_init (void)
{
  PORTB->PCR0 = (1<<8) /* GPIO                  */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* puddselect= 0         */
              ;
  PORTB->PCR1 = (1<<8) /* GPIO                  */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* puddselect= 0         */
              ;

  GPIOB->PDDR = (1 << 1) | (1 << 0); /* PTB0, PTB1 : Output    */
  GPIOB->PSOR = (1 << 0);            /* PTB0: Set  : Light off */
  GPIOB->PCOR = (1 << 1);            /* PTB1: Clear: Output 0  */
}
