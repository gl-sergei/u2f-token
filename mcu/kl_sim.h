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
static struct SIM *const SIM = (struct SIM *const)0x40047000;
