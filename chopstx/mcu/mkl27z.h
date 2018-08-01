/* System Integration Module.  */
struct SIM {
  volatile uint32_t SOPT1;    /* System Options Register 1               */
  volatile uint32_t SOPT1CFG; /* SOPT1 Configuration Register            */
  uint32_t reserved0[1023];   /*                                         */
  volatile uint32_t SOPT2;    /* System Options Register 2               */
  uint32_t reserved1[1];      /*                                         */
  volatile uint32_t SOPT4;    /* System Options Register 4               */
  volatile uint32_t SOPT5;    /* System Options Register 5               */
  uint32_t reserved2[1];      /*                                         */
  volatile uint32_t SOPT7;    /* System Options Register 7               */
  uint32_t reserved3[2];       /*                                         */
  volatile uint32_t SDID;     /* System Device Identification Register   */
  uint32_t reserved4[3];      /*                                         */
  volatile uint32_t SCGC4;    /* System Clock Gating Control Register 4  */
  volatile uint32_t SCGC5;    /* System Clock Gating Control Register 5  */
  volatile uint32_t SCGC6;    /* System Clock Gating Control Register 6  */
  volatile uint32_t SCGC7;    /* System Clock Gating Control Register 7  */
  volatile uint32_t CLKDIV1;  /* System Clock Divider Register 1         */
  uint32_t reserved5[1];      /*                                         */
  volatile uint32_t FCFG1;    /* Flash Configuration Register 1          */
  volatile uint32_t FCFG2;    /* Flash Configuration Register 2          */
  uint32_t reserved6[1];      /*                                         */
  volatile uint32_t UIDMH;    /* Unique Identification Register Mid-High */
  volatile uint32_t UIDML;    /* Unique Identification Register Mid Low  */
  volatile uint32_t UIDL;     /* Unique Identification Register Low      */
  uint32_t reserved7[39];     /*                                         */
  volatile uint32_t COPC;     /* COP Control Register                    */
  volatile uint32_t SRVCOP;   /* Service COP                             */
};

/* Port control. */
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

struct GPIO {
  volatile uint32_t PDOR; /* Port Data Output Register    */
  volatile uint32_t PSOR; /* Port Set Output Register     */
  volatile uint32_t PCOR; /* Port Clear Output Register   */
  volatile uint32_t PTOR; /* Port Toggle Output Register  */
  volatile uint32_t PDIR; /* Port Data Input Register     */
  volatile uint32_t PDDR; /* Port Data Direction Register */
};

static struct SIM *const SIM = (struct SIM *)0x40047000;
static struct PORT *const PORTB = (struct PORT *)0x4004A000;
static struct PORT *const PORTD = (struct PORT *)0x4004C000;
static struct PORT *const PORTE = (struct PORT *)0x4004D000;
static struct GPIO *const GPIOB = (struct GPIO *)0x400FF040;
static struct GPIO *const GPIOD = (struct GPIO *)0x400FF0C0;
static struct GPIO *const GPIOE = (struct GPIO *)0x400FF100;
