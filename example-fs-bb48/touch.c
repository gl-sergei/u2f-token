#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <mcu/mkl27z.h>

struct TPM {
  volatile uint32_t SC;
  volatile uint32_t CNT;
  volatile uint32_t MOD;
  volatile uint32_t C0SC;
  volatile uint32_t C0V;
  volatile uint32_t C1SC;
  volatile uint32_t C1V;
  uint32_t rsvd0[13];
  volatile uint32_t STATUS;
  uint32_t rsvd1[7];
  volatile uint32_t POL;
  uint32_t rsvd2[4];
  volatile uint32_t CONF;
};

static struct TPM *const TPM1 = (struct TPM *const)0x40039000;

static chopstx_intr_t tpm1_intr;
#define INTR_REQ_TPM1 18

uint16_t
touch_get (void)
{
  /* Assert LOW.  */ 
  PORTB->PCR1 = (1<<8) /* GPIO                  */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* puddselect= 0         */
              ;

  /* Let the register to pull it up.  */
  PORTB->PCR1 = (1<<3) /* TPM1_CH1              */
              | (0<<6) /* DriveStrengthEnable=0 */
              | (0<<4) /* PassiveFilterEnable=0 */
              | (1<<2) /* SlewRateEnable = slow */
              | (0<<1) /* pull enable = 0       */ 
              | (0<<0) /* puddselect= 0         */
              ;

  chopstx_intr_wait (&tpm1_intr);
  /* Clear overflow.  */
  TPM1->SC |= 0x80;

  return TPM1->C1V;
}


void
touch_init (void)
{
  chopstx_claim_irq (&tpm1_intr, INTR_REQ_TPM1);

  /* TOF clear, TOIE=1, CPWMS=0, CMOD=1, PS=000.  */
  TPM1->SC = 0xc4;

  /* Input capture mode: MSB = 0, MSA = 0 */
  /*        Rising edge: ELSB=0 ELSA=1 */
  TPM1->C1SC = 0x82;
  TPM1->POL=0;

  /* Triggered by TPM1_CH1.   */
  /*   channel 1: TRGSEL=0010 */
  /*   external:  TRGSRC=0    */
  /*   active low:TRGPOL=1    */
  /* stop on overflow: CSOO=1 */ 
  /* start on trigger: CSOT=1 */
  TPM1->CONF = 0x02c30000;

  touch_get ();
}
