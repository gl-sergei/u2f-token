/*
 * Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware for STM32
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
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
 * recipients of GNU GPL by a written offer.
 *
 */

/* Bit fields for CMU HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_AES    (0x1UL << 0)  /* AES Accelerator */
#define CMU_HFCORECLKEN0_DMA    (0x1UL << 1)  /* DMA Controller */
#define CMU_HFCORECLKEN0_LE     (0x1UL << 2)  /* Low Energy Peripheral Interface */
#define CMU_HFCORECLKEN0_USBC   (0x1UL << 3)  /* USB Interface Core */
#define CMU_HFCORECLKEN0_USB    (0x1UL << 4)  /* USB Interface */

/* Bit fields for CMU HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER0  (0x1UL << 0)  /* Timer 0 */
#define CMU_HFPERCLKEN0_TIMER1  (0x1UL << 1)  /* Timer 1 */
#define CMU_HFPERCLKEN0_TIMER2  (0x1UL << 2)  /* Timer 2 */
#define CMU_HFPERCLKEN0_USART0  (0x1UL << 3)  /* USART 0 */
#define CMU_HFPERCLKEN0_USART1  (0x1UL << 4)  /* USART 1 */
#define CMU_HFPERCLKEN0_ACMP0   (0x1UL << 5)  /* Analog Comparator 0 */
#define CMU_HFPERCLKEN0_PRS     (0x1UL << 6)  /* Peripheral Reflex System */
#define CMU_HFPERCLKEN0_IDAC0   (0x1UL << 7)  /* Current DAC 0 */
#define CMU_HFPERCLKEN0_GPIO    (0x1UL << 8)  /* General purpose IO */
#define CMU_HFPERCLKEN0_VCMP    (0x1UL << 9)  /* Voltage Comparator */
#define CMU_HFPERCLKEN0_ADC0    (0x1UL << 10) /* ADC 0 */
#define CMU_HFPERCLKEN0_I2C0    (0x1UL << 11) /* I2C 0 */

/* Bit fields for CMU OSCENCMD */
#define CMU_OSCENCMD_HFRCOEN      (0x1UL << 0)   /* HFRCO Enable */
#define CMU_OSCENCMD_HFRCODIS     (0x1UL << 1)   /* HFRCO Disable */
#define CMU_OSCENCMD_HFXOEN       (0x1UL << 2)   /* HFXO Enable */
#define CMU_OSCENCMD_HFXODIS      (0x1UL << 3)   /* HFXO Disable */
#define CMU_OSCENCMD_AUXHFRCOEN   (0x1UL << 4)   /* AUXHFRCO Enable */
#define CMU_OSCENCMD_AUXHFRCODIS  (0x1UL << 5)   /* AUXHFRCO Disable */
#define CMU_OSCENCMD_LFRCOEN      (0x1UL << 6)   /* LFRCO Enable */
#define CMU_OSCENCMD_LFRCODIS     (0x1UL << 7)   /* LFRCO Disable */
#define CMU_OSCENCMD_LFXOEN       (0x1UL << 8)   /* LFXO Enable */
#define CMU_OSCENCMD_LFXODIS      (0x1UL << 9)   /* LFXO Disable */
#define CMU_OSCENCMD_USHFRCOEN    (0x1UL << 10)  /* USHFRCO Enable */
#define CMU_OSCENCMD_USHFRCODIS   (0x1UL << 11)  /* USHFRCO Disable */

/* Bit fields for CMU LFCCLKEN0 */
#define CMU_LFCCLKEN0_USBLE       (0x1UL << 0)   /* USB LE Clock Clock Enable */

/* Bit fields for CMU USBCRCTRL */
#define CMU_USBCRCTRL_EN          (0x1UL << 0)   /* Clock Recovery Enable */
#define CMU_USBCRCTRL_LSMODE      (0x1UL << 1)   /* Low Speed Clock Recovery Mode */

/* Bit fields for CMU STATUS */
#define CMU_STATUS_HFRCOENS       (0x1UL << 0)   /* HFRCO Enable Status */
#define CMU_STATUS_HFRCORDY       (0x1UL << 1)   /* HFRCO Ready */
#define CMU_STATUS_HFXOENS        (0x1UL << 2)   /* HFXO Enable Status */
#define CMU_STATUS_HFXORDY        (0x1UL << 3)   /* HFXO Ready */
#define CMU_STATUS_AUXHFRCOENS    (0x1UL << 4)   /* AUXHFRCO Enable Status */
#define CMU_STATUS_AUXHFRCORDY    (0x1UL << 5)   /* AUXHFRCO Ready */
#define CMU_STATUS_LFRCOENS       (0x1UL << 6)   /* LFRCO Enable Status */
#define CMU_STATUS_LFRCORDY       (0x1UL << 7)   /* LFRCO Ready */
#define CMU_STATUS_LFXOENS        (0x1UL << 8)   /* LFXO Enable Status */
#define CMU_STATUS_LFXORDY        (0x1UL << 9)   /* LFXO Ready */
#define CMU_STATUS_HFRCOSEL       (0x1UL << 10)  /* HFRCO Selected */
#define CMU_STATUS_HFXOSEL        (0x1UL << 11)  /* HFXO Selected */
#define CMU_STATUS_LFRCOSEL       (0x1UL << 12)  /* LFRCO Selected */
#define CMU_STATUS_LFXOSEL        (0x1UL << 13)  /* LFXO Selected */
#define CMU_STATUS_CALBSY         (0x1UL << 14)  /* Calibration Busy */
#define CMU_STATUS_USBCLFXOSEL    (0x1UL << 16)  /* USBC LFXO Selected */
#define CMU_STATUS_USBCLFRCOSEL   (0x1UL << 17)  /* USBC LFRCO Selected */
#define CMU_STATUS_USBCUSHFRCOSEL (0x1UL << 18)  /* USBC USHFRCO Selected */
#define CMU_STATUS_USBCHFCLKSYNC  (0x1UL << 20)  /* USBC is synchronous to HFCLK */
#define CMU_STATUS_USHFRCOENS     (0x1UL << 21)  /* USHFRCO Enable Status */
#define CMU_STATUS_USHFRCORDY     (0x1UL << 22)  /* USHFRCO Ready */
#define CMU_STATUS_USHFRCOSUSPEND (0x1UL << 23)  /* USHFRCO is suspended */
#define CMU_STATUS_USHFRCODIV2SEL (0x1UL << 26)  /* USHFRCODIV2 Selected */

/* Bit fields for CMU USHFRCOCONF */
#define CMU_USHFRCOCONF_BAND_48MHZ     (1 << 0)     /* mode 48MHZ for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_BAND_24MHZ     (3 << 0)     /* mode 24MHZ for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_USHFRCODIV2DIS (0x1UL << 4) /* USHFRCO divider for HFCLK disable */

/* Bit fields for CMU CMD */
#define CMU_CMD_HFCLKSEL_HFRCO       (0x1UL << 0)   /* mode HFRCO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_HFXO        (0x2UL << 0)   /* mode HFXO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_LFRCO       (0x3UL << 0)   /* mode LFRCO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_LFXO        (0x4UL << 0)   /* mode LFXO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_USHFRCODIV2 (0x5UL << 0)   /* mode USHFRCODIV2 for CMU_CMD */
#define CMU_CMD_CALSTART             (0x1UL << 3)   /* Calibration Start */
#define CMU_CMD_CALSTOP              (0x1UL << 4)   /* Calibration Stop */
#define CMU_CMD_USBCCLKSEL_LFXO      (0x2UL << 5)   /* mode LFXO for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_LFRCO     (0x2UL << 5)   /* mode LFRCO for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_USHFRCO   (0x4UL << 5)   /* mode USHFRCO for CMU_CMD */

/* Bit fields for USB CTRL */
#define USB_CTRL_DMPUAP             (0x1UL << 1)   /* DMPU Active Polarity */
#define USB_CTRL_LEMOSCCTRL_NONE    (0x01UL << 4)  /* mode NONE for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_GATE    (0x1UL << 4)   /* mode GATE for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_SUSPEND (0x2UL << 4)   /* mode SUSPEND for USB_CTRL */
#define USB_CTRL_LEMPHYCTRL         (0x1UL << 7)   /* Low Energy Mode USB PHY Control */
#define USB_CTRL_LEMIDLEEN          (0x1UL << 9)   /* Low Energy Mode on Bus Idle Enable */
#define USB_CTRL_LEMNAKEN           (0x1UL << 10)  /* Low Energy Mode on OUT NAK Enable */
#define USB_CTRL_LEMADDRMEN         (0x1UL << 11)  /* Low Energy Mode on Device Address Mismatch Enable */
#define USB_CTRL_VREGDIS            (0x1UL << 16)  /* Voltage Regulator Disable */
#define USB_CTRL_VREGOSEN           (0x1UL << 17)  /* VREGO Sense Enable */

/* Bit fields for USB STATUS */
#define USB_STATUS_VREGOS           (0x1UL << 0)   /* VREGO Sense Output */
#define USB_STATUS_LEMACTIVE        (0x1UL << 2)   /* Low Energy Mode Active */

/* Bit fields for USB IFS */
#define USB_IFS_VREGOSH             (0x1UL << 0)   /* Set VREGO Sense High Interrupt Flag */
#define USB_IFS_VREGOSL             (0x1UL << 1)   /* Set VREGO Sense Low Interrupt Flag */

/* Bit fields for USB IFC */
#define USB_IFC_VREGOSH             (0x1UL << 0)   /* Clear VREGO Sense High Interrupt Flag */
#define USB_IFC_VREGOSL             (0x1UL << 1)   /* Clear VREGO Sense Low Interrupt Flag */

/* Bit fields for USB IEN */
#define USB_IEN_VREGOSH             (0x1UL << 0)   /* VREGO Sense High Interrupt Enable */
#define USB_IEN_VREGOSL             (0x1UL << 1)   /* VREGO Sense Low Interrupt Enable */

/* Bit fields for USB ROUTE */
#define USB_ROUTE_PHYPEN            (0x1UL << 0)   /* USB PHY Pin Enable */
#define USB_ROUTE_DMPUPEN           (0x1UL << 2)   /* DMPU Pin Enable */

/* Bit fields for USB DCFG */
#define USB_DCFG_DEVSPD_MASK        0x3UL                /* Bit mask for USB_DEVSPD */
#define USB_DCFG_DEVSPD_LS          (0x00000002UL << 0)  /* mode LS for USB_DCFG */
#define USB_DCFG_DEVSPD_FS          (0x00000003UL << 0)  /* mode FS for USB_DCFG */
#define USB_DCFG_NZSTSOUTHSHK       (0x1UL << 2)         /* Non-Zero-Length Status OUT Handshake */
#define USB_DCFG_ENA32KHZSUSP       (0x1UL << 3)         /* Enable 32 KHz Suspend mode */
#define USB_DCFG_DEVADDR_MASK       0x7F0UL              /* Bit mask for USB_DEVADDR */
#define USB_DCFG_PERFRINT_MASK      0x1800UL             /* Bit mask for USB_PERFRINT */
#define USB_DCFG_PERFRINT_80PCNT    (0x00000000UL << 11) /* mode 80PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_85PCNT    (0x00000001UL << 11) /* mode 85PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_90PCNT    (0x00000002UL << 11) /* mode 90PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_95PCNT    (0x00000003UL << 11) /* mode 95PCNT for USB_DCFG */
#define USB_DCFG_ERRATICINTMSK      (0x1UL << 15)        /*  */
#define USB_DCFG_RESVALID_MASK      0xFC000000UL         /* Bit mask for USB_RESVALID */

/* Bit fields for USB DCTL */
#define USB_DCTL_MASK               0x00018FFFUL         /* Mask for USB_DCTL */
#define USB_DCTL_RMTWKUPSIG         (0x1UL << 0)         /* Remote Wakeup Signaling */
#define USB_DCTL_SFTDISCON          (0x1UL << 1)         /* Soft Disconnect */
#define USB_DCTL_GNPINNAKSTS        (0x1UL << 2)         /* Global Non-periodic IN NAK Status */
#define USB_DCTL_GOUTNAKSTS         (0x1UL << 3)         /* Global OUT NAK Status */
#define USB_DCTL_TSTCTL_MASK        0x70UL               /* Bit mask for USB_TSTCTL */
#define USB_DCTL_TSTCTL_DISABLE     (0x00000000UL << 4)  /* mode DISABLE for USB_DCTL */
#define USB_DCTL_TSTCTL_J           (0x00000001UL << 4)  /* mode J for USB_DCTL */
#define USB_DCTL_TSTCTL_K           (0x00000002UL << 4)  /* mode K for USB_DCTL */
#define USB_DCTL_TSTCTL_SE0NAK      (0x00000003UL << 4)  /* mode SE0NAK for USB_DCTL */
#define USB_DCTL_TSTCTL_PACKET      (0x00000004UL << 4)  /* mode PACKET for USB_DCTL */
#define USB_DCTL_TSTCTL_FORCE       (0x00000005UL << 4)  /* mode FORCE for USB_DCTL */
#define USB_DCTL_SGNPINNAK          (0x1UL << 7)         /* Set Global Non-periodic IN NAK */
#define USB_DCTL_CGNPINNAK          (0x1UL << 8)         /* Clear Global Non-periodic IN NAK */
#define USB_DCTL_SGOUTNAK           (0x1UL << 9)         /* Set Global OUT NAK */
#define USB_DCTL_CGOUTNAK           (0x1UL << 10)        /* Clear Global OUT NAK */
#define USB_DCTL_PWRONPRGDONE       (0x1UL << 11)        /* Power-On Programming Done */
#define USB_DCTL_IGNRFRMNUM         (0x1UL << 15)        /* Ignore Frame number For Isochronous End points */
#define USB_DCTL_NAKONBBLE          (0x1UL << 16)        /* NAK on Babble Error */

/* Bit fields for USB PCGCCTL */
#define USB_PCGCCTL_STOPPCLK        (0x1UL << 0)         /* Stop PHY clock */
#define USB_PCGCCTL_GATEHCLK        (0x1UL << 1)         /* Gate HCLK */
#define USB_PCGCCTL_PWRCLMP         (0x1UL << 2)         /* Power Clamp */
#define USB_PCGCCTL_RSTPDWNMODULE   (0x1UL << 3)         /* Reset Power-Down Modules */
#define USB_PCGCCTL_PHYSLEEP        (0x1UL << 6)         /* PHY In Sleep */

/* Bit fields for USB GRSTCTL */
#define USB_GRSTCTL_MASK            0xC00007F3UL         /* Mask for USB_GRSTCTL */
#define USB_GRSTCTL_CSFTRST         (0x1UL << 0)         /* Core Soft Reset */
#define USB_GRSTCTL_PIUFSSFTRST     (0x1UL << 1)         /* PIU FS Dedicated Controller Soft Reset */
#define USB_GRSTCTL_RXFFLSH         (0x1UL << 4)         /* RxFIFO Flush */
#define USB_GRSTCTL_TXFFLSH         (0x1UL << 5)         /* TxFIFO Flush */
#define USB_GRSTCTL_TXFNUM_MASK     0x7C0UL              /* Bit mask for USB_TXFNUM */
#define USB_GRSTCTL_TXFNUM_F0       (0x00000000UL << 6)  /* mode F0 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F1       (0x00000001UL << 6)  /* mode F1 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F2       (0x00000002UL << 6)  /* mode F2 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F3       (0x00000003UL << 6)  /* mode F3 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F4       (0x00000004UL << 6)  /* mode F4 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F5       (0x00000005UL << 6)  /* mode F5 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F6       (0x00000006UL << 6)  /* mode F6 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_FALL     (0x00000010UL << 6)  /* mode FALL for USB_GRSTCTL */
#define USB_GRSTCTL_DMAREQ          (0x1UL << 30)        /* DMA Request Signal */
#define USB_GRSTCTL_AHBIDLE         (0x1UL << 31)        /* AHB Master Idle */

/* Bit fields for USB GINTSTS */
#define USB_GINTSTS_CURMOD       (0x1UL << 0)   /* Current Mode of Operation */
#define USB_GINTSTS_SOF          (0x1UL << 3)   /* Start of Frame */
#define USB_GINTSTS_RXFLVL       (0x1UL << 4)   /* RxFIFO Non-Empty */
#define USB_GINTSTS_GINNAKEFF    (0x1UL << 6)   /* Global IN Non-periodic NAK Effective */
#define USB_GINTSTS_GOUTNAKEFF   (0x1UL << 7)   /* Global OUT NAK Effective */
#define USB_GINTSTS_ERLYSUSP     (0x1UL << 10)  /* Early Suspend */
#define USB_GINTSTS_USBSUSP      (0x1UL << 11)  /* USB Suspend */
#define USB_GINTSTS_USBRST       (0x1UL << 12)  /* USB Reset */
#define USB_GINTSTS_ENUMDONE     (0x1UL << 13)  /* Enumeration Done */
#define USB_GINTSTS_ISOOUTDROP   (0x1UL << 14)  /* Isochronous OUT Packet Dropped Interrupt */
#define USB_GINTSTS_EOPF         (0x1UL << 15)  /* End of Periodic Frame Interrupt */
#define USB_GINTSTS_IEPINT       (0x1UL << 18)  /* IN Endpoints Interrupt */
#define USB_GINTSTS_OEPINT       (0x1UL << 19)  /* OUT Endpoints Interrupt */
#define USB_GINTSTS_INCOMPISOIN  (0x1UL << 20)  /* Incomplete Isochronous IN Transfer */
#define USB_GINTSTS_INCOMPLP     (0x1UL << 21)  /* Incomplete Periodic Transfer */
#define USB_GINTSTS_FETSUSP      (0x1UL << 22)  /* Data Fetch Suspended */
#define USB_GINTSTS_RESETDET     (0x1UL << 23)  /* Reset detected Interrupt */
#define USB_GINTSTS_WKUPINT      (0x1UL << 31)  /* Resume/Remote Wakeup Detected Interrupt */

/* Bit fields for USB GAHBCFG */
#define USB_GAHBCFG_GLBLINTRMSK           (0x1UL << 0)          /* Global Interrupt Mask */
#define USB_GAHBCFG_HBSTLEN_MASK          0x1EUL                /* Bit mask for USB_HBSTLEN */
#define USB_GAHBCFG_HBSTLEN_SINGLE        (0x00000000UL << 1)   /* mode SINGLE for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR          (0x00000001UL << 1)   /* mode INCR for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR4         (0x00000003UL << 1)   /* mode INCR4 for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR8         (0x00000005UL << 1)   /* mode INCR8 for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR16        (0x00000007UL << 1)   /* mode INCR16 for USB_GAHBCFG */
#define USB_GAHBCFG_DMAEN                 (0x1UL << 5)          /* DMA Enable */
#define USB_GAHBCFG_NPTXFEMPLVL           (0x1UL << 7)          /* Non-Periodic TxFIFO Empty Level */
#define USB_GAHBCFG_NPTXFEMPLVL_MASK      0x80UL                /* Bit mask for USB_NPTXFEMPLVL */
#define USB_GAHBCFG_NPTXFEMPLVL_HALFEMPTY (0x00000000UL << 7)   /* mode HALFEMPTY for USB_GAHBCFG */
#define USB_GAHBCFG_NPTXFEMPLVL_EMPTY     (0x00000001UL << 7)   /* mode EMPTY for USB_GAHBCFG */
#define USB_GAHBCFG_REMMEMSUPP            (0x1UL << 21)         /* Remote Memory Support */
#define USB_GAHBCFG_NOTIALLDMAWRIT        (0x1UL << 22)         /* Notify All DMA Writes */
#define USB_GAHBCFG_AHBSINGLE             (0x1UL << 23)         /* AHB Single Support */

/* Bit fields for USB GINTMSK */
#define USB_GINTMSK_MODEMISMSK            (0x1UL << 1)    /* Mode Mismatch Interrupt Mask */
#define USB_GINTMSK_SOFMSK                (0x1UL << 3)    /* Start of Frame Mask */
#define USB_GINTMSK_RXFLVLMSK             (0x1UL << 4)    /* Receive FIFO Non-Empty Mask */
#define USB_GINTMSK_GINNAKEFFMSK          (0x1UL << 6)    /* Global Non-periodic IN NAK Effective Mask */
#define USB_GINTMSK_GOUTNAKEFFMSK         (0x1UL << 7)    /* Global OUT NAK Effective Mask */
#define USB_GINTMSK_ERLYSUSPMSK           (0x1UL << 10)   /* Early Suspend Mask */
#define USB_GINTMSK_USBSUSPMSK            (0x1UL << 11)   /* USB Suspend Mask */
#define USB_GINTMSK_USBRSTMSK             (0x1UL << 12)   /* USB Reset Mask */
#define USB_GINTMSK_ENUMDONEMSK           (0x1UL << 13)   /* Enumeration Done Mask */
#define USB_GINTMSK_ISOOUTDROPMSK         (0x1UL << 14)   /* Isochronous OUT Packet Dropped Interrupt Mask */
#define USB_GINTMSK_EOPFMSK               (0x1UL << 15)   /* End of Periodic Frame Interrupt Mask */
#define USB_GINTMSK_IEPINTMSK             (0x1UL << 18)   /* IN Endpoints Interrupt Mask */
#define USB_GINTMSK_OEPINTMSK             (0x1UL << 19)   /* OUT Endpoints Interrupt Mask */
#define USB_GINTMSK_INCOMPISOINMSK        (0x1UL << 20)   /* Incomplete Isochronous IN Transfer Mask */
#define USB_GINTMSK_INCOMPLPMSK           (0x1UL << 21)   /* Incomplete Periodic Transfer Mask */
#define USB_GINTMSK_FETSUSPMSK            (0x1UL << 22)   /* Data Fetch Suspended Mask */
#define USB_GINTMSK_RESETDETMSK           (0x1UL << 23)   /* Reset detected Interrupt Mask */
#define USB_GINTMSK_WKUPINTMSK            (0x1UL << 31)   /* Resume/Remote Wakeup Detected Interrupt Mask */

/* Bit fields for USB DAINTMSK */
#define USB_DAINTMSK_INEPMSK0      (0x1UL << 0)   /* IN Endpoint 0 Interrupt mask Bit */
#define USB_DAINTMSK_INEPMSK1      (0x1UL << 1)   /* IN Endpoint 1 Interrupt mask Bit */
#define USB_DAINTMSK_INEPMSK2      (0x1UL << 2)   /* IN Endpoint 2 Interrupt mask Bit */
#define USB_DAINTMSK_INEPMSK3      (0x1UL << 3)   /* IN Endpoint 3 Interrupt mask Bit */
#define USB_DAINTMSK_OUTEPMSK0     (0x1UL << 16)  /* OUT Endpoint 0 Interrupt mask Bit */
#define USB_DAINTMSK_OUTEPMSK1     (0x1UL << 17)  /* OUT Endpoint 1 Interrupt mask Bit */
#define USB_DAINTMSK_OUTEPMSK2     (0x1UL << 18)  /* OUT Endpoint 2 Interrupt mask Bit */
#define USB_DAINTMSK_OUTEPMSK3     (0x1UL << 19)  /* OUT Endpoint 3 Interrupt mask Bit */

/* Bit fields for USB DIEPMSK */
#define USB_DIEPMSK_XFERCOMPLMSK    (0x1UL << 0)  /* Transfer Completed Interrupt Mask */
#define USB_DIEPMSK_EPDISBLDMSK     (0x1UL << 1)  /* Endpoint Disabled Interrupt Mask */
#define USB_DIEPMSK_AHBERRMSK       (0x1UL << 2)  /* AHB Error Mask */
#define USB_DIEPMSK_TIMEOUTMSK      (0x1UL << 3)  /* Timeout Condition Mask */
#define USB_DIEPMSK_INTKNTXFEMPMSK  (0x1UL << 4)  /* IN Token Received When TxFIFO Empty Mask */
#define USB_DIEPMSK_INEPNAKEFFMSK   (0x1UL << 6)  /* IN Endpoint NAK Effective Mask */
#define USB_DIEPMSK_TXFIFOUNDRNMSK  (0x1UL << 8)  /* Fifo Underrun Mask */
#define USB_DIEPMSK_NAKMSK          (0x1UL << 13) /* NAK interrupt Mask */

/* Bit fields for USB DOEPMSK */
#define USB_DOEPMSK_XFERCOMPLMSK    (0x1UL << 0)  /* Transfer Completed Interrupt Mask */
#define USB_DOEPMSK_EPDISBLDMSK     (0x1UL << 1)  /* Endpoint Disabled Interrupt Mask */
#define USB_DOEPMSK_AHBERRMSK       (0x1UL << 2)  /* AHB Error */
#define USB_DOEPMSK_SETUPMSK        (0x1UL << 3)  /* SETUP Phase Done Mask */
#define USB_DOEPMSK_OUTTKNEPDISMSK  (0x1UL << 4)  /* OUT Token Received when Endpoint Disabled Mask */
#define USB_DOEPMSK_STSPHSERCVDMSK  (0x1UL << 5)  /* Status Phase Received Mask */
#define USB_DOEPMSK_BACK2BACKSETUP  (0x1UL << 6)  /* Back-to-Back SETUP Packets Received Mask */
#define USB_DOEPMSK_OUTPKTERRMSK    (0x1UL << 8)  /* OUT Packet Error Mask */
#define USB_DOEPMSK_BBLEERRMSK      (0x1UL << 12) /* Babble Error interrupt Mask */
#define USB_DOEPMSK_NAKMSK          (0x1UL << 13) /* NAK interrupt Mask */

/* Bit fields for USB DIEP_CTL */
#define USB_DIEP_CTL_USBACTEP           (0x1UL << 15)        /* USB Active Endpoint */
#define USB_DIEP_CTL_DPIDEOF            (0x1UL << 16)        /* Endpoint Data PID / Even or Odd Frame */
#define USB_DIEP_CTL_DPIDEOF_DATA0EVEN  (0 << 16)            /* mode DATA0EVEN for USB_DIEP_CTL */
#define USB_DIEP_CTL_DPIDEOF_DATA1ODD   (1 << 16)            /* mode DATA1ODD for USB_DIEP_CTL */
#define USB_DIEP_CTL_NAKSTS             (0x1UL << 17)        /* NAK Status */
#define USB_DIEP_CTL_NAKSTS_MASK        0x20000UL            /* Bit mask for USB_NAKSTS */
#define USB_DIEP_CTL_EPTYPE_MASK        0xC0000UL            /* Bit mask for USB_EPTYPE */
#define USB_DIEP_CTL_EPTYPE_CONTROL     (0x00000000UL << 18) /* mode CONTROL for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_ISO         (0x00000001UL << 18) /* mode ISO for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_BULK        (0x00000002UL << 18) /* mode BULK for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_INT         (0x00000003UL << 18) /* mode INT for USB_DIEP_CTL */
#define USB_DIEP_CTL_STALL              (0x1UL << 21)        /* Handshake */
#define USB_DIEP_CTL_TXFNUM_SHIFT       22                   /* Shift value for USB_TXFNUM */
#define USB_DIEP_CTL_TXFNUM_MASK        0x3C00000UL          /* Bit mask for USB_TXFNUM */
#define USB_DIEP_CTL_CNAK               (0x1UL << 26)        /* Clear NAK */
#define USB_DIEP_CTL_SNAK               (0x1UL << 27)        /* Set NAK */
#define USB_DIEP_CTL_SETD0PIDEF         (0x1UL << 28)        /* Set DATA0 PID / Even Frame */
#define USB_DIEP_CTL_SETD1PIDOF         (0x1UL << 29)        /* Set DATA1 PID / Odd Frame */
#define USB_DIEP_CTL_EPDIS              (0x1UL << 30)        /* Endpoint Disable */
#define USB_DIEP_CTL_EPENA              (0x1UL << 31)        /* Endpoint Enable */

/* Bit fields for USB DIEP_INT */
#define USB_DIEP_INT_MASK          0x000038DFUL   /* Mask for USB_DIEP_INT */
#define USB_DIEP_INT_XFERCOMPL     (0x1UL << 0)   /* Transfer Completed Interrupt */
#define USB_DIEP_INT_EPDISBLD      (0x1UL << 1)   /* Endpoint Disabled Interrupt */
#define USB_DIEP_INT_AHBERR        (0x1UL << 2)   /* AHB Error */
#define USB_DIEP_INT_TIMEOUT       (0x1UL << 3)   /* Timeout Condition */
#define USB_DIEP_INT_INTKNTXFEMP   (0x1UL << 4)   /* IN Token Received When TxFIFO is Empty */
#define USB_DIEP_INT_INEPNAKEFF    (0x1UL << 6)   /* IN Endpoint NAK Effective */
#define USB_DIEP_INT_TXFEMP        (0x1UL << 7)   /* Transmit FIFO Empty */
#define USB_DIEP_INT_PKTDRPSTS     (0x1UL << 11)  /* Packet Drop Status */
#define USB_DIEP_INT_BBLEERR       (0x1UL << 12)  /* NAK Interrupt */
#define USB_DIEP_INT_NAKINTRPT     (0x1UL << 13)  /* NAK Interrupt */

/* Bit fields for USB DIEP0CTL */
#define USB_DIEP0CTL_MPS_MASK       0x3UL                /* Bit mask for USB_MPS */
#define USB_DIEP0CTL_MPS_64B        (0x00000000UL << 0)  /* mode 64B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_32B        (0x00000001UL << 0)  /* mode 32B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_16B        (0x00000002UL << 0)  /* mode 16B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_8B         (0x00000003UL << 0)  /* mode 8B for USB_DIEP0CTL */
#define USB_DIEP0CTL_USBACTEP       (0x1UL << 15)        /* USB Active Endpoint */
#define USB_DIEP0CTL_NAKSTS         (0x1UL << 17)        /* NAK Status */
#define USB_DIEP0CTL_STALL          (0x1UL << 21)        /* Handshake */
#define USB_DIEP0CTL_TXFNUM_SHIFT   22                   /* Shift value for USB_TXFNUM */
#define USB_DIEP0CTL_TXFNUM_MASK    0x3C00000UL          /* Bit mask for USB_TXFNUM */
#define USB_DIEP0CTL_CNAK           (0x1UL << 26)        /* Clear NAK */
#define USB_DIEP0CTL_SNAK           (0x1UL << 27)        /* Set NAK */
#define USB_DIEP0CTL_EPDIS          (0x1UL << 30)        /* Endpoint Disable */
#define USB_DIEP0CTL_EPENA          (0x1UL << 31)        /* Endpoint Enable */

/* Bit fields for USB DIEP0INT */
#define USB_DIEP0INT_XFERCOMPL    (0x1UL << 0)   /* Transfer Completed Interrupt */
#define USB_DIEP0INT_EPDISBLD     (0x1UL << 1)   /* Endpoint Disabled Interrupt */
#define USB_DIEP0INT_AHBERR       (0x1UL << 2)   /* AHB Error */
#define USB_DIEP0INT_TIMEOUT      (0x1UL << 3)   /* Timeout Condition */
#define USB_DIEP0INT_INTKNTXFEMP  (0x1UL << 4)   /* IN Token Received When TxFIFO is Empty */
#define USB_DIEP0INT_INEPNAKEFF   (0x1UL << 6)   /* IN Endpoint NAK Effective */
#define USB_DIEP0INT_TXFEMP       (0x1UL << 7)   /* Transmit FIFO Empty */
#define USB_DIEP0INT_PKTDRPSTS    (0x1UL << 11)  /* Packet Drop Status */
#define USB_DIEP0INT_BBLEERR      (0x1UL << 12)  /* NAK Interrupt */
#define USB_DIEP0INT_NAKINTRPT    (0x1UL << 13)  /* NAK Interrupt */

/* Bit fields for USB DOEP_CTL */
#define USB_DOEP_CTL_MPS_MASK       0x7FFUL              /* Bit mask for USB_MPS */
#define USB_DOEP_CTL_USBACTEP       (0x1UL << 15)        /* USB Active Endpoint */
#define USB_DOEP_CTL_DPIDEOF        (0x1UL << 16)        /* Endpoint Data PID / Even-odd Frame */
#define USB_DOEP_CTL_NAKSTS         (0x1UL << 17)        /* NAK Status */
#define USB_DOEP_CTL_EPTYPE_SHIFT   18                   /* Shift value for USB_EPTYPE */
#define USB_DOEP_CTL_EPTYPE_MASK    0xC0000UL            /* Bit mask for USB_EPTYPE */
#define USB_DOEP_CTL_EPTYPE_CONTROL (0x00000000UL << 18) /* mode CONTROL for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_ISO     (0x00000001UL << 18) /* mode ISO for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_BULK    (0x00000002UL << 18) /* mode BULK for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_INT     (0x00000003UL << 18) /* mode INT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SNP            (0x1UL << 20)        /* Snoop Mode */
#define USB_DOEP_CTL_STALL          (0x1UL << 21)        /* STALL Handshake */
#define USB_DOEP_CTL_CNAK           (0x1UL << 26)        /* Clear NAK */
#define USB_DOEP_CTL_SNAK           (0x1UL << 27)        /* Set NAK */
#define USB_DOEP_CTL_SETD0PIDEF     (0x1UL << 28)        /* Set DATA0 PID / Even Frame */
#define USB_DOEP_CTL_SETD1PIDOF     (0x1UL << 29)        /* Set DATA1 PID / Odd Frame */
#define USB_DOEP_CTL_EPDIS          (0x1UL << 30)        /* Endpoint Disable */
#define USB_DOEP_CTL_EPENA          (0x1UL << 31)        /* Endpoint Enable */

/* Bit fields for USB DOEP_INT */
#define USB_DOEP_INT_XFERCOMPL       (0x1UL << 0)   /* Transfer Completed Interrupt */
#define USB_DOEP_INT_EPDISBLD        (0x1UL << 1)   /* Endpoint Disabled Interrupt */
#define USB_DOEP_INT_AHBERR          (0x1UL << 2)   /* AHB Error */
#define USB_DOEP_INT_SETUP           (0x1UL << 3)   /* Setup Phase Done */
#define USB_DOEP_INT_OUTTKNEPDIS     (0x1UL << 4)   /* OUT Token Received When Endpoint Disabled */
#define USB_DOEP_INT_STSPHSERCVD     (0x1UL << 5)   /* Status Phase Received For Control Write */
#define USB_DOEP_INT_BACK2BACKSETUP  (0x1UL << 6)   /* Back-to-Back SETUP Packets Received */
#define USB_DOEP_INT_PKTDRPSTS       (0x1UL << 11)  /* Packet Drop Status */
#define USB_DOEP_INT_BBLEERR         (0x1UL << 12)  /* Babble Error */
#define USB_DOEP_INT_NAKINTRPT       (0x1UL << 13)  /* NAK Interrupt */
#define USB_DOEP_INT_STUPPKTRCVD     (0x1UL << 15)  /* Setup Packet Received */

/* Bit fields for USB DOEP0CTL */
#define USB_DOEP0CTL_MPS_64B        (0x00000000UL << 0)  /* mode 64B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_32B        (0x00000001UL << 0)  /* mode 32B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_16B        (0x00000002UL << 0)  /* mode 16B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_8B         (0x00000003UL << 0)  /* mode 8B for USB_DOEP0CTL */
#define USB_DOEP0CTL_USBACTEP       (0x1UL << 15)        /* USB Active Endpoint */
#define USB_DOEP0CTL_NAKSTS         (0x1UL << 17)        /* NAK Status */
#define USB_DOEP0CTL_SNP            (0x1UL << 20)        /* Snoop Mode */
#define USB_DOEP0CTL_STALL          (0x1UL << 21)        /* Handshake */
#define USB_DOEP0CTL_CNAK           (0x1UL << 26)        /* Clear NAK */
#define USB_DOEP0CTL_SNAK           (0x1UL << 27)        /* Set NAK */
#define USB_DOEP0CTL_EPDIS          (0x1UL << 30)        /* Endpoint Disable */
#define USB_DOEP0CTL_EPENA          (0x1UL << 31)        /* Endpoint Enable */

/* Bit fields for USB DOEP0INT */
#define USB_DOEP0INT_XFERCOMPL       (0x1UL << 0)   /* Transfer Completed Interrupt */
#define USB_DOEP0INT_EPDISBLD        (0x1UL << 1)   /* Endpoint Disabled Interrupt */
#define USB_DOEP0INT_AHBERR          (0x1UL << 2)   /* AHB Error */
#define USB_DOEP0INT_SETUP           (0x1UL << 3)   /* Setup Phase Done */
#define USB_DOEP0INT_OUTTKNEPDIS     (0x1UL << 4)   /* OUT Token Received When Endpoint Disabled */
#define USB_DOEP0INT_STSPHSERCVD     (0x1UL << 5)   /* Status Phase Received For Control Write */
#define USB_DOEP0INT_BACK2BACKSETUP  (0x1UL << 6)   /* Back-to-Back SETUP Packets Received */
#define USB_DOEP0INT_PKTDRPSTS       (0x1UL << 11)  /* Packet Drop Status */
#define USB_DOEP0INT_BBLEERR         (0x1UL << 12)  /* NAK Interrupt */
#define USB_DOEP0INT_NAKINTRPT       (0x1UL << 13)  /* NAK Interrupt */
#define USB_DOEP0INT_STUPPKTRCVD     (0x1UL << 15)  /* Setup Packet Received */

/* Bit fields for USB DOEP0TSIZ */
#define USB_DOEP0TSIZ_XFERSIZE_SHIFT 0             /* Shift value for USB_XFERSIZE */
#define USB_DOEP0TSIZ_XFERSIZE_MASK  0x7FUL        /* Bit mask for USB_XFERSIZE */
#define USB_DOEP0TSIZ_PKTCNT_SHIFT   19            /* Shift value for USB_PKTCNT */
#define USB_DOEP0TSIZ_PKTCNT_MASK    0x80000UL     /* Bit mask for USB_PKTCNT */
#define USB_DOEP0TSIZ_SUPCNT_SHIFT   29            /* Shift value for USB_SUPCNT */
#define USB_DOEP0TSIZ_SUPCNT_MASK    0x60000000UL  /* Bit mask for USB_SUPCNT */

/* Bit fields for USB GRXSTSP */
#define USB_GRXSTSP_CHEPNUM_SHIFT     0                     /* Shift value for USB_CHEPNUM */
#define USB_GRXSTSP_CHEPNUM_MASK      0xFUL                 /* Bit mask for USB_CHEPNUM */
#define USB_GRXSTSP_BCNT_SHIFT        4                     /* Shift value for USB_BCNT */
#define USB_GRXSTSP_BCNT_MASK         0x7FF0UL              /* Bit mask for USB_BCNT */
#define USB_GRXSTSP_DPID_SHIFT        15                    /* Shift value for USB_DPID */
#define USB_GRXSTSP_DPID_DATA0        (0x00000000UL << 15)  /* mode DATA0 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DATA1        (0x00000001UL << 15)  /* mode DATA1 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DATA2        (0x00000002UL << 15)  /* mode DATA2 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_MDATA        (0x00000003UL << 15)  /* mode MDATA for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_MASK       0x1E0000UL            /* Bit mask for USB_PKTSTS */
#define USB_GRXSTSP_PKTSTS_GOUTNAK    (0x00000001UL << 17)  /* mode GOUTNAK for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_PKTRCV     (0x00000002UL << 17)  /* mode PKTRCV for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_XFERCOMPL  (0x00000003UL << 17)  /* mode XFERCOMPL for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_SETUPCOMPL (0x00000004UL << 17)  /* mode SETUPCOMPL for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_TGLERR     (0x00000005UL << 17)  /* mode TGLERR for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_SETUPRCV   (0x00000006UL << 17)  /* mode SETUPRCV for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_CHLT       (0x00000007UL << 17)  /* mode CHLT for USB_GRXSTSP */
#define USB_GRXSTSP_FN_SHIFT          21                    /* Shift value for USB_FN */
#define USB_GRXSTSP_FN_MASK           0x1E00000UL           /* Bit mask for USB_FN */

/* Bit fields for ADC CMD */
#define ADC_CMD_SINGLESTART   (0x1UL << 0)  /* Single Conversion Start */
#define ADC_CMD_SINGLESTOP    (0x1UL << 1)  /* Single Conversion Stop */
#define ADC_CMD_SCANSTART     (0x1UL << 2)  /* Scan Sequence Start */
#define ADC_CMD_SCANSTOP      (0x1UL << 3)  /* Scan Sequence Stop */

/* Bit fields for ADC STATUS */
#define ADC_STATUS_SINGLEACT        (0x1UL << 0)          /* Single Conversion Active */
#define ADC_STATUS_SCANACT          (0x1UL << 1)          /* Scan Conversion Active */
#define ADC_STATUS_SINGLEREFWARM    (0x1UL << 8)          /* Single Reference Warmed Up */
#define ADC_STATUS_SCANREFWARM      (0x1UL << 9)          /* Scan Reference Warmed Up */
#define ADC_STATUS_WARM             (0x1UL << 12)         /* ADC Warmed Up */
#define ADC_STATUS_SINGLEDV         (0x1UL << 16)         /* Single Sample Data Valid */
#define ADC_STATUS_SCANDV           (0x1UL << 17)         /* Scan Data Valid */
#define ADC_STATUS_SCANDATASRC_CH0  (0x00000000UL << 24)  /* CH0 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH1  (0x00000001UL << 24)  /* CH1 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH2  (0x00000002UL << 24)  /* CH2 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH3  (0x00000003UL << 24)  /* CH3 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH4  (0x00000004UL << 24)  /* CH4 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH5  (0x00000005UL << 24)  /* CH5 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH6  (0x00000006UL << 24)  /* CH6 scan is ready */
#define ADC_STATUS_SCANDATASRC_CH7  (0x00000007UL << 24)  /* CH7 scan is ready */

/* Bit fields for ADC IEN */
#define ADC_IEN_SINGLE       (0x1UL << 0)  /* Single Conversion Complete */
#define ADC_IEN_SCAN         (0x1UL << 1)  /* Scan Conversion Complete */
#define ADC_IEN_SINGLEOF     (0x1UL << 8)  /* Single Result Overflow */
#define ADC_IEN_SCANOF       (0x1UL << 9)  /* Scan Result Overflow */

/* Bit fields for ADC IF */
#define ADC_IF_SINGLE        (0x1UL << 0)  /* Single Conversion Complete */
#define ADC_IF_SCAN          (0x1UL << 1)  /* Scan Conversion Complete */
#define ADC_IF_SINGLEOF      (0x1UL << 8)  /* Single Result Overflow */
#define ADC_IF_SCANOF        (0x1UL << 9)  /* Scan Result Overflow */

/* Bit fields for ADC IFS */
#define ADC_IFS_SINGLE       (0x1UL << 0)  /* Single Conversion Complete */
#define ADC_IFS_SCAN         (0x1UL << 1)  /* Scan Conversion Complete */
#define ADC_IFS_SINGLEOF     (0x1UL << 8)  /* Single Result Overflow */
#define ADC_IFS_SCANOF       (0x1UL << 9)  /* Scan Result Overflow */

/* Bit fields for ADC IFC */
#define ADC_IFC_SINGLE       (0x1UL << 0)  /* Single Conversion Complete */
#define ADC_IFC_SCAN         (0x1UL << 1)  /* Scan Conversion Complete */
#define ADC_IFC_SINGLEOF     (0x1UL << 8)  /* Single Result Overflow */
#define ADC_IFC_SCANOF       (0x1UL << 9)  /* Scan Result Overflow */

/* Limits imposed by the USB peripheral */
#define NP_RX_QUE_DEPTH       8
#define HP_RX_QUE_DEPTH       8
#define MAX_XFER_LEN          524287L         /* 2^19 - 1 bytes */
#define MAX_PACKETS_PR_XFER   1023            /* 2^10 - 1 packets */
#define MAX_NUM_TX_FIFOS      3               /* In addition to EP0 Tx FIFO */
#define MAX_NUM_IN_EPS        3               /* In addition to EP0 */
#define MAX_NUM_OUT_EPS       3               /* In addition to EP0 */
#define MAX_DEVICE_FIFO_SIZE_INWORDS 384U
#define MIN_EP_FIFO_SIZE_INWORDS  16U         /* Unit is words (32bit) */
#define MIN_EP_FIFO_SIZE_INBYTES  64U         /* Unit is bytes (8bit) */

/* Bit fields for MSC WRITECTRL */
#define MSC_WRITECTRL_WREN           (0x1UL << 0) /* Enable Write/Erase Controller */
#define MSC_WRITECTRL_IRQERASEABORT  (0x1UL << 1) /* Abort Page Erase on Interrupt */

/* Bit fields for MSC TIMEBASE */
#define MSC_TIMEBASE_PERIOD_1US  (0x00000000UL << 16)  /* mode 1US for MSC_TIMEBASE */
#define MSC_TIMEBASE_PERIOD_5US  (0x00000001UL << 16)  /* mode 5US for MSC_TIMEBASE */

/* Bit fields for MSC WRITECMD */
#define MSC_WRITECMD_LADDRIM     (0x1UL << 0)  /* Load MSC_ADDRB into ADDR */
#define MSC_WRITECMD_ERASEPAGE   (0x1UL << 1)  /* Erase Page */
#define MSC_WRITECMD_WRITEEND    (0x1UL << 2)  /* End Write Mode */
#define MSC_WRITECMD_WRITEONCE   (0x1UL << 3)  /* Word Write-Once Trigger */
#define MSC_WRITECMD_WRITETRIG   (0x1UL << 4)  /* Word Write Sequence Trigger */
#define MSC_WRITECMD_ERASEABORT  (0x1UL << 5)  /* Abort erase sequence */
#define MSC_WRITECMD_ERASEMAIN0  (0x1UL << 8)  /* Mass erase region 0 */
#define MSC_WRITECMD_CLEARWDATA  (0x1UL << 12) /* Clear WDATA state */

/* Bit fields for MSC STATUS */
#define MSC_STATUS_BUSY         (0x1UL << 0)  /* Erase/Write Busy */
#define MSC_STATUS_LOCKED       (0x1UL << 1)  /* Access Locked */
#define MSC_STATUS_INVADDR      (0x1UL << 2)  /* Invalid Write Address or Erase Page */
#define MSC_STATUS_WDATAREADY   (0x1UL << 3)  /* WDATA Write Ready */
#define MSC_STATUS_WORDTIMEOUT  (0x1UL << 4)  /* Flash Write Word Timeout */
#define MSC_STATUS_ERASEABORTED (0x1UL << 5)  /* The Current Flash Erase Operation Aborted */
#define MSC_STATUS_PCRUNNING    (0x1UL << 6)  /* Performance Counters Running */

#define MSC_UNLOCK_CODE      0x1B71 /* MSC unlock code */

struct CMU
{
  volatile uint32_t CTRL;          /* CMU Control Register */
  volatile uint32_t HFCORECLKDIV;  /* High Frequency Core Clock Division Register */
  volatile uint32_t HFPERCLKDIV;   /* High Frequency Peripheral Clock Division Register */
  volatile uint32_t HFRCOCTRL;     /* HFRCO Control Register */
  volatile uint32_t LFRCOCTRL;     /* LFRCO Control Register */
  volatile uint32_t AUXHFRCOCTRL;  /* AUXHFRCO Control Register */
  volatile uint32_t CALCTRL;       /* Calibration Control Register */
  volatile uint32_t CALCNT;        /* Calibration Counter Register */
  volatile uint32_t OSCENCMD;      /* Oscillator Enable/Disable Command Register */
  volatile uint32_t CMD;           /* Command Register */
  volatile uint32_t LFCLKSEL;      /* Low Frequency Clock Select Register */
  volatile const uint32_t STATUS;  /* Status Register */
  volatile const uint32_t IF;      /* Interrupt Flag Register */
  volatile uint32_t IFS;           /* Interrupt Flag Set Register */
  volatile uint32_t IFC;           /* Interrupt Flag Clear Register */
  volatile uint32_t IEN;           /* Interrupt Enable Register */
  volatile uint32_t HFCORECLKEN0;  /* High Frequency Core Clock Enable Register 0 */
  volatile uint32_t HFPERCLKEN0;   /* High Frequency Peripheral Clock Enable Register 0 */
  uint32_t RESERVED0[2];           /* Reserved for future use */
  volatile const uint32_t SYNCBUSY;/* Synchronization Busy Register */
  volatile uint32_t FREEZE;        /* Freeze Register */
  volatile uint32_t LFACLKEN0;     /* Low Frequency A Clock Enable Register 0  (Async Reg) */
  uint32_t RESERVED1[1];           /* Reserved for future use */
  volatile uint32_t LFBCLKEN0;     /* Low Frequency B Clock Enable Register 0 (Async Reg) */
  volatile uint32_t LFCCLKEN0;     /* Low Frequency C Clock Enable Register 0 (Async Reg) */
  volatile uint32_t LFAPRESC0;     /* Low Frequency A Prescaler Register 0 (Async Reg) */
  uint32_t RESERVED2[1];           /* Reserved for future use */
  volatile uint32_t LFBPRESC0;     /* Low Frequency B Prescaler Register 0  (Async Reg) */
  uint32_t RESERVED3[1];           /* Reserved for future use */
  volatile uint32_t PCNTCTRL;      /* PCNT Control Register */

  uint32_t RESERVED4[1];           /* Reserved for future use */
  volatile uint32_t ROUTE;         /* I/O Routing Register */
  volatile uint32_t LOCK;          /* Configuration Lock Register */

  uint32_t RESERVED5[18];          /* Reserved for future use */
  volatile uint32_t USBCRCTRL;     /* USB Clock Recovery Control */
  volatile uint32_t USHFRCOCTRL;   /* USHFRCO Control */
  volatile uint32_t USHFRCOTUNE;   /* USHFRCO Frequency Tune */
  volatile uint32_t USHFRCOCONF;   /* USHFRCO Configuration */
};

#define CMU_BASE 0x400C8000UL            /* CMU base address */
#define CMU ((struct CMU *) CMU_BASE)    /* CMU base pointer */


struct GPIO_P
{
  volatile uint32_t CTRL;              /* Port Control Register */
  volatile uint32_t MODEL;             /* Port Pin Mode Low Register */
  volatile uint32_t MODEH;             /* Port Pin Mode High Register */
  volatile uint32_t DOUT;              /* Port Data Out Register */
  volatile uint32_t DOUTSET;           /* Port Data Out Set Register */
  volatile uint32_t DOUTCLR;           /* Port Data Out Clear Register */
  volatile uint32_t DOUTTGL;           /* Port Data Out Toggle Register */
  volatile const uint32_t DIN;         /* Port Data In Register */
  volatile uint32_t PINLOCKN;          /* Port Unlocked Pins Register */
};

struct GPIO
{
  struct GPIO_P P[6];                  /* Port configuration bits */

  uint32_t RESERVED0[10];              /* Reserved for future use */
  volatile uint32_t EXTIPSELL;         /* External Interrupt Port Select Low Register */
  volatile uint32_t EXTIPSELH;         /* External Interrupt Port Select High Register */
  volatile uint32_t EXTIRISE;          /* External Interrupt Rising Edge Trigger Register */
  volatile uint32_t EXTIFALL;          /* External Interrupt Falling Edge Trigger Register */
  volatile uint32_t IEN;               /* Interrupt Enable Register */
  volatile const uint32_t IF;          /* Interrupt Flag Register */
  volatile uint32_t IFS;               /* Interrupt Flag Set Register */
  volatile uint32_t IFC;               /* Interrupt Flag Clear Register */

  volatile uint32_t ROUTE;             /* I/O Routing Register */
  volatile uint32_t INSENSE;           /* Input Sense Register */
  volatile uint32_t LOCK;              /* Configuration Lock Register */
  volatile uint32_t CTRL;              /* GPIO Control Register */
  volatile uint32_t CMD;               /* GPIO Command Register */
  volatile uint32_t EM4WUEN;           /* EM4 Wake-up Enable Register */
  volatile uint32_t EM4WUPOL;          /* EM4 Wake-up Polarity Register */
  volatile const uint32_t EM4WUCAUSE;  /* EM4 Wake-up Cause Register */
};

#define GPIO_BASE 0x40006000UL             /* GPIO base address */
#define GPIO ((struct GPIO *) GPIO_BASE)   /* GPIO base pointer */


struct TIMER_CC
{
  volatile uint32_t CTRL;          /* CC Channel Control Register */
  volatile uint32_t CCV;           /* CC Channel Value Register */
  volatile const uint32_t CCVP;    /* CC Channel Value Peek Register */
  volatile uint32_t CCVB;          /* CC Channel Buffer Register */
};

struct TIMER
{
  volatile uint32_t CTRL;          /* Control Register */
  volatile uint32_t CMD;           /* Command Register */
  volatile const uint32_t STATUS;  /* Status Register */
  volatile uint32_t IEN;           /* Interrupt Enable Register */
  volatile const uint32_t IF;      /* Interrupt Flag Register */
  volatile uint32_t IFS;           /* Interrupt Flag Set Register */
  volatile uint32_t IFC;           /* Interrupt Flag Clear Register */
  volatile uint32_t TOP;           /* Counter Top Value Register */
  volatile uint32_t TOPB;          /* Counter Top Value Buffer Register */
  volatile uint32_t CNT;           /* Counter Value Register */
  volatile uint32_t ROUTE;         /* I/O Routing Register */

  uint32_t RESERVED0[1];           /* Reserved registers */
  struct TIMER_CC CC[3];           /* Compare/Capture Channel */

  uint32_t RESERVED1[4];           /* Reserved for future use */
  volatile uint32_t DTCTRL;        /* DTI Control Register */
  volatile uint32_t DTTIME;        /* DTI Time Control Register */
  volatile uint32_t DTFC;          /* DTI Fault Configuration Register */
  volatile uint32_t DTOGEN;        /* DTI Output Generation Enable Register */
  volatile const uint32_t DTFAULT; /* DTI Fault Register */
  volatile uint32_t DTFAULTC;     /* DTI Fault Clear Register */
  volatile uint32_t DTLOCK;        /* DTI Configuration Lock Register */
};

#define TIMER0_BASE 0x40010000UL                /* TIMER0 base address */
#define TIMER1_BASE 0x40010400UL                /* TIMER1 base address */
#define TIMER2_BASE 0x40010800UL                /* TIMER2 base address */
#define TIMER0 ((struct TIMER *) TIMER0_BASE)   /* TIMER0 base pointer */
#define TIMER1 ((struct TIMER *) TIMER1_BASE)   /* TIMER1 base pointer */
#define TIMER2 ((struct TIMER *) TIMER2_BASE)   /* TIMER2 base pointer */


struct PRS_CH
{
  volatile uint32_t CTRL;          /* Channel Control Register */
};

struct PRS
{
  volatile uint32_t SWPULSE;       /* Software Pulse Register */
  volatile uint32_t SWLEVEL;       /* Software Level Register */
  volatile uint32_t ROUTE;         /* I/O Routing Register */

  uint32_t RESERVED0[1];           /* Reserved registers*/
  struct PRS_CH CH[6];             /* Channel registers*/

  uint32_t RESERVED1[6];           /* Reserved for future use */
  volatile uint32_t TRACECTRL;     /* MTB Trace Control Register */
};

#define PRS_BASE 0x400CC000UL             /* PRS base address */
#define PRS ((struct PRS *) PRS_BASE)     /* PRS base pointer */


struct ACMP
{
  volatile uint32_t CTRL;          /* Control Register */
  volatile uint32_t INPUTSEL;      /* Input Selection Register */
  volatile const uint32_t STATUS;  /* Status Register */
  volatile uint32_t IEN;           /* Interrupt Enable Register */
  volatile const uint32_t IF;      /* Interrupt Flag Register */
  volatile uint32_t IFS;           /* Interrupt Flag Set Register */
  volatile uint32_t IFC;           /* Interrupt Flag Clear Register */
  volatile uint32_t ROUTE;         /* I/O Routing Register */
};

#define ACMP0_BASE 0x40001000UL              /* ACMP0 base address */
#define ACMP0 ((struct ACMP *) ACMP0_BASE)   /* ACMP0 base pointer */


struct USB_DIEP
{
  volatile uint32_t CTL;          /* Device IN Endpoint x+1 Control Register */
  uint32_t RESERVED0[1];          /* Reserved for future use */
  volatile uint32_t INT;          /* Device IN Endpoint x+1 Interrupt Register */
  uint32_t RESERVED1[1];          /* Reserved for future use */
  volatile uint32_t TSIZ;         /* Device IN Endpoint x+1 Transfer Size Register */
  volatile uint32_t DMAADDR;      /* Device IN Endpoint x+1 DMA Address Register */
  volatile const uint32_t TXFSTS; /* Device IN Endpoint x+1 Transmit FIFO Status Register */
  uint32_t RESERVED2[1];          /* Reserved future */
};

struct USB_DOEP
{
  volatile uint32_t CTL;          /* Device OUT Endpoint x+1 Control Register */
  uint32_t RESERVED0[1];          /* Reserved for future use */
  volatile uint32_t INT;          /* Device OUT Endpoint x+1 Interrupt Register */
  uint32_t RESERVED1[1];          /* Reserved for future use */
  volatile uint32_t TSIZ;         /* Device OUT Endpoint x+1 Transfer Size Register */
  volatile uint32_t DMAADDR;      /* Device OUT Endpoint x+1 DMA Address Register */
  uint32_t RESERVED2[2];          /* Reserved future */
};

struct USB
{
  volatile uint32_t CTRL;              /* System Control Register */
  volatile const uint32_t STATUS;      /* System Status Register */
  volatile const uint32_t IF;          /* Interrupt Flag Register */
  volatile uint32_t IFS;               /* Interrupt Flag Set Register */
  volatile uint32_t IFC;               /* Interrupt Flag Clear Register */
  volatile uint32_t IEN;               /* Interrupt Enable Register */
  volatile uint32_t ROUTE;             /* I/O Routing Register */

  uint32_t RESERVED0[61435];           /* Reserved for future use */
  volatile uint32_t GAHBCFG;           /* AHB Configuration Register */
  volatile uint32_t GUSBCFG;           /* USB Configuration Register */
  volatile uint32_t GRSTCTL;           /* Reset Register */
  volatile uint32_t GINTSTS;           /* Interrupt Register */
  volatile uint32_t GINTMSK;           /* Interrupt Mask Register */
  volatile const uint32_t GRXSTSR;     /* Receive Status Debug Read Register */
  volatile const uint32_t GRXSTSP;     /* Receive Status Read and Pop Register */
  volatile uint32_t GRXFSIZ;           /* Receive FIFO Size Register */
  volatile uint32_t GNPTXFSIZ;         /* Non-periodic Transmit FIFO Size Register */

  uint32_t RESERVED1[12];              /* Reserved for future use */
  volatile uint32_t GDFIFOCFG;         /* Global DFIFO Configuration Register */

  uint32_t RESERVED2[41];              /* Reserved for future use */
  volatile uint32_t DIEPTXF1;          /* Device IN Endpoint Transmit FIFO 1 Size Register */
  volatile uint32_t DIEPTXF2;          /* Device IN Endpoint Transmit FIFO 2 Size Register */
  volatile uint32_t DIEPTXF3;          /* Device IN Endpoint Transmit FIFO 3 Size Register */

  uint32_t RESERVED3[444];             /* Reserved for future use */
  volatile uint32_t DCFG;              /* Device Configuration Register */
  volatile uint32_t DCTL;              /* Device Control Register */
  volatile const uint32_t DSTS;        /* Device Status Register */
  uint32_t RESERVED4[1];               /* Reserved for future use */
  volatile uint32_t DIEPMSK;           /* Device IN Endpoint Common Interrupt Mask Register */
  volatile uint32_t DOEPMSK;           /* Device OUT Endpoint Common Interrupt Mask Register */
  volatile const uint32_t DAINT;       /* Device All Endpoints Interrupt Register */
  volatile uint32_t DAINTMSK;          /* Device All Endpoints Interrupt Mask Register */

  uint32_t RESERVED5[5];               /* Reserved for future use */
  volatile uint32_t DIEPEMPMSK;        /* Device IN Endpoint FIFO Empty Interrupt Mask Register */

  uint32_t RESERVED6[50];              /* Reserved for future use */
  volatile uint32_t DIEP0CTL;          /* Device IN Endpoint 0 Control Register */
  uint32_t RESERVED7[1];               /* Reserved for future use */
  volatile uint32_t DIEP0INT;          /* Device IN Endpoint 0 Interrupt Register */
  uint32_t RESERVED8[1];               /* Reserved for future use */
  volatile uint32_t DIEP0TSIZ;         /* Device IN Endpoint 0 Transfer Size Register */
  volatile uint32_t DIEP0DMAADDR;      /* Device IN Endpoint 0 DMA Address Register */
  volatile const uint32_t DIEP0TXFSTS; /* Device IN Endpoint 0 Transmit FIFO Status Register */

  uint32_t RESERVED9[1];               /* Reserved registers */
  struct USB_DIEP DIEP[3];             /* Device IN Endpoint x+1 Registers */

  uint32_t RESERVED10[96];             /* Reserved for future use */
  volatile uint32_t DOEP0CTL;          /* Device OUT Endpoint 0 Control Register */
  uint32_t RESERVED11[1];              /* Reserved for future use */
  volatile uint32_t DOEP0INT;          /* Device OUT Endpoint 0 Interrupt Register */
  uint32_t RESERVED12[1];              /* Reserved for future use */
  volatile uint32_t DOEP0TSIZ;         /* Device OUT Endpoint 0 Transfer Size Register */
  volatile uint32_t DOEP0DMAADDR;      /* Device OUT Endpoint 0 DMA Address Register */

  uint32_t RESERVED13[2];              /* Reserved registers */
  struct USB_DOEP DOEP[3];             /* Device OUT Endpoint x+1 Registers */

  uint32_t RESERVED14[160];            /* Reserved for future use */
  volatile uint32_t PCGCCTL;           /* Power and Clock Gating Control Register */

  uint32_t RESERVED15[127];            /* Reserved registers */
  volatile uint32_t FIFO0D[384];       /* Device EP 0 FIFO */

  uint32_t RESERVED16[640];            /* Reserved registers */
  volatile uint32_t FIFO1D[384];       /* Device EP 1 FIFO */

  uint32_t RESERVED17[640];            /* Reserved registers */
  volatile uint32_t FIFO2D[384];       /* Device EP 2 FIFO */

  uint32_t RESERVED18[640];            /* Reserved registers */
  volatile uint32_t FIFO3D[384];       /* Device EP 3 FIFO */

  uint32_t RESERVED19[28288];          /* Reserved registers */
  volatile uint32_t FIFORAM[512];      /* Direct Access to Data FIFO RAM for Debugging (2 KB) */
};

#define USB_BASE 0x400C4000UL                /* USB base address */
#define USB ((struct USB *) USB_BASE)        /* USB base pointer */

#define USB_DINEPS   ((struct USB_DIEP *) &USB->DIEP0CTL)
#define USB_DOUTEPS  ((struct USB_DOEP *) &USB->DOEP0CTL)
#define USB_FIFOS    ((uint32_t *) &USB->FIFO0D)
#define USB_DIEPTXFS ((uint32_t *) &USB->DIEPTXF1)


struct ROMTABLE
{
  volatile const uint32_t PID4;        /* JEP_106_BANK */
  volatile const uint32_t PID5;        /* Unused */
  volatile const uint32_t PID6;        /* Unused */
  volatile const uint32_t PID7;        /* Unused */
  volatile const uint32_t PID0;        /* Chip family LSB, chip major revision */
  volatile const uint32_t PID1;        /* JEP_106_NO, Chip family MSB */
  volatile const uint32_t PID2;        /* Chip minor rev MSB, JEP_106_PRESENT, JEP_106_NO */
  volatile const uint32_t PID3;        /* Chip minor rev LSB */
  volatile const uint32_t CID0;        /* Unused */
};

#define ROMTABLE_BASE 0xF00FFFD0UL                   /* ROMTABLE base address */
#define ROMTABLE ((struct ROMTABLE *) ROMTABLE_BASE) /* ROMTABLE base pointer */

struct ADC
{
  volatile uint32_t CTRL;                /* Control Register */
  volatile uint32_t CMD;                 /* Command Register */
  volatile const uint32_t STATUS;        /* Status Register */
  volatile uint32_t SINGLECTRL;          /* Single Sample Control Register */
  volatile uint32_t SCANCTRL;            /* Scan Control Register */
  volatile uint32_t IEN;                 /* Interrupt Enable Register */
  volatile const uint32_t IF;            /* Interrupt Flag Register */
  volatile uint32_t IFS;                 /* Interrupt Flag Set Register */
  volatile uint32_t IFC;                 /* Interrupt Flag Clear Register */
  volatile const uint32_t SINGLEDATA;    /* Single Conversion Result Data */
  volatile const uint32_t SCANDATA;      /* Scan Conversion Result Data */
  volatile const uint32_t SINGLEDATAP;   /* Single Conversion Result Data Peek Register */
  volatile const uint32_t SCANDATAP;     /* Scan Sequence Result Data Peek Register */
  volatile uint32_t CAL;                 /* Calibration Register */

  uint32_t RESERVED0[1];                 /* Reserved for future use */
  volatile uint32_t BIASPROG;            /* Bias Programming Register */
};

#define ADC0_BASE (0x40002000UL)         /* ADC0 base address */
#define ADC0 ((struct ADC *) ADC0_BASE)  /* ADC0 base pointer */

struct DEVINFO
{
  volatile const uint32_t CAL;          /* Calibration temperature and checksum */
  volatile const uint32_t ADC0CAL0;     /* ADC0 Calibration register 0 */
  volatile const uint32_t ADC0CAL1;     /* ADC0 Calibration register 1 */
  volatile const uint32_t ADC0CAL2;     /* ADC0 Calibration register 2 */
  uint32_t RESERVED0[2];                /* Reserved */
  volatile const uint32_t IDAC0CAL0;    /* IDAC0 calibration register */
  volatile const uint32_t USHFRCOCAL0;  /* USHFRCO calibration register */
  uint32_t RESERVED1[1];                /* Reserved */
  volatile const uint32_t AUXHFRCOCAL0; /* AUXHFRCO calibration register 0 */
  volatile const uint32_t AUXHFRCOCAL1; /* AUXHFRCO calibration register 1 */
  volatile const uint32_t HFRCOCAL0;    /* HFRCO calibration register 0 */
  volatile const uint32_t HFRCOCAL1;    /* HFRCO calibration register 1 */
  volatile const uint32_t MEMINFO;      /* Memory information */
  uint32_t RESERVED2[2];                /* Reserved */
  volatile const uint32_t UNIQUEL;      /* Low 32 bits of device unique number */
  volatile const uint32_t UNIQUEH;      /* High 32 bits of device unique number */
  volatile const uint32_t MSIZE;        /* Flash and SRAM Memory size in KiloBytes */
  volatile const uint32_t PART;         /* Part description */
};

#define DEVINFO_BASE (0x0FE081B0UL)               /* DEVINFO base address */
#define DEVINFO ((struct DEVINFO *) DEVINFO_BASE) /* DEVINFO base pointer */

struct MSC
{
  volatile uint32_t CTRL;               /* Memory System Control Register  */
  volatile uint32_t READCTRL;           /* Read Control Register  */
  volatile uint32_t WRITECTRL;          /* Write Control Register  */
  volatile uint32_t WRITECMD;           /* Write Command Register  */
  volatile uint32_t ADDRB;              /* Page Erase/Write Address Buffer  */

  uint32_t RESERVED0[1];                /* Reserved for future use **/
  volatile uint32_t WDATA;              /* Write Data Register  */
  volatile const uint32_t STATUS;       /* Status Register  */

  uint32_t RESERVED1[3];                /* Reserved for future use **/
  volatile const uint32_t IF;           /* Interrupt Flag Register  */
  volatile uint32_t IFS;                /* Interrupt Flag Set Register  */
  volatile uint32_t IFC;                /* Interrupt Flag Clear Register  */
  volatile uint32_t IEN;                /* Interrupt Enable Register  */
  volatile uint32_t LOCK;               /* Configuration Lock Register  */
  volatile uint32_t CMD;                /* Command Register  */
  volatile const uint32_t CACHEHITS;    /* Cache Hits Performance Counter  */
  volatile const uint32_t CACHEMISSES;  /* Cache Misses Performance Counter  */
  uint32_t RESERVED2[1];                /* Reserved for future use **/
  volatile uint32_t TIMEBASE;           /* Flash Write and Erase Timebase  */
  volatile uint32_t MASSLOCK;           /* Mass Erase Lock Register  */
  volatile uint32_t IRQLATENCY;         /* Irq Latency Register  */
};

#define MSC_BASE (0x400C0000UL)        /* MSC base address */
#define MSC ((struct MSC *) MSC_BASE)  /* MSC base pointer */


struct WDOG
{
  volatile uint32_t CTRL;              /* Control Register */
  volatile uint32_t CMD;               /* Command Register */
  const volatile uint32_t SYNCBUSY;    /* Synchronization Busy Register */
};

#define WDOG_BASE (0x40088000UL)         /* WDOG base address */
#define WDOG ((struct WDOG *) WDOG_BASE) /* WDOG base pointer */


struct SCB
{
  volatile const uint32_t CPUID;  /* Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;         /* Offset: 0x004 (R/W)  Interrupt Control and State Register */
  volatile uint32_t VTOR;         /* Offset: 0x008 (R/W)  Vector Table Offset Register */
  volatile uint32_t AIRCR;        /* Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;          /* Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;          /* Offset: 0x014 (R/W)  Configuration Control Register */
  uint32_t RESERVED1;
  volatile uint32_t SHP[2U];      /* Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED */
  volatile uint32_t SHCSR;        /* Offset: 0x024 (R/W)  System Handler Control and State Register */
};

#define SCB_BASE (0XE000ED00UL)       /* System Control Block Base address */
#define SCB ((struct SCB*) SCB_BASE)  /* SCB configuration struct */
