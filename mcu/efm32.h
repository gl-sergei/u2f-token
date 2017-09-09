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
#define CMU_HFPERCLKEN0_GPIO    (0x1UL << 8)  /* General purpose IO */
#define CMU_HFPERCLKEN0_VCMP    (0x1UL << 9)  /* Voltage Comparator */
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

/* Bit fields for USB ROUTE */
#define USB_ROUTE_PHYPEN            (0x1UL << 0)   /* USB PHY Pin Enable */
#define USB_ROUTE_DMPUPEN           (0x1UL << 2)   /* DMPU Pin Enable */

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
