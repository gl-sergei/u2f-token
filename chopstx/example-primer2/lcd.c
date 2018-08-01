#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "sys.h" /* for set_led */
#include "st7732.h"
#include "board.h"

#define PERIPH_BASE	0x40000000
#define APBPERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x10000)

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

static struct GPIO *const GPIO_LCD = ((struct GPIO *const) GPIO_LED_BASE);
static struct GPIO *const GPIO_LCD_CTRL = ((struct GPIO *const) GPIO_USB_BASE);

#define GPIO_LCD_RD	4
#define GPIO_LCD_WR	5
#define GPIO_LCD_RST	6
#define GPIO_LCD_CS	7
#define GPIO_LCD_RS	11
/* PE7:LCD_D0 - PE14:LCD_D7 */
#define GPIO_DATA_SHIFT	7
#define GPIO_DATA_MASK (0xff << GPIO_DATA_SHIFT)

static void
lcd_command_common (st7732_cmd_t cmd)
{
  /* Set command. */
  GPIO_LCD->BRR = GPIO_DATA_MASK & ~(cmd << GPIO_DATA_SHIFT);
  GPIO_LCD->BSRR = GPIO_DATA_MASK & (cmd << GPIO_DATA_SHIFT);
  /* Set CMD mode.  */
  GPIO_LCD_CTRL->BRR = (1 << GPIO_LCD_RS);
  /* Asert /CS.  */
  GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_CS);
  /* Asert /WR.  */
  GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_WR);
  // chopstx_usec_wait (1);
  /* Negate /WR.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_WR);

  /* Return DATA mode.  */
  GPIO_LCD_CTRL->BSRR = (1 << GPIO_LCD_RS);
}

/* Issue command with no data read/write.  */
void
lcd_command_no (st7732_cmd_t cmd)
{
  lcd_command_common (cmd);

  /* Negate /CS.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_CS);
}

#if 0
void
lcd_command_readn (st7732_cmd_t cmd, uint8_t *data, size_t n)
{
  volatile int dummy __attribute__ ((unused));

  lcd_command_common (cmd);

  /* Set GPIO_LCD to input mode.  */
  GPIO_LCD->CRH = 0x88888888;
  GPIO_LCD->CRL = 0x88888833;

  /* Assert /RD.  */
  GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_RD);
  // chopstx_usec_wait (1);
  /* Dummy read.  */
  dummy = GPIO_LCD->IDR;
  /* Negate /RD.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_RD);

  /* Read loop.  */
  while (n-- > 0)
    {
      /* Assert /RD.  */
      GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_RD);
      // chopstx_usec_wait (1);
      /* Negate /RD.  */
      GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_RD);
      *data++ = GPIO_LCD->IDR >> GPIO_DATA_SHIFT;
    }
	  
  /* Negate /CS.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_CS);

  /* Set GPIO_LCD to output mode.  */
  GPIO_LCD->CRH = 0x83333333;
  GPIO_LCD->CRL = 0x38888833;
}
#endif

/* Issue command with N data write.  */
void
lcd_command_writen (st7732_cmd_t cmd, uint8_t *data, size_t n)
{
  lcd_command_common (cmd);

  /* Write loop.  */
  while (n-- > 0)
    {
      uint8_t b = *data++;

      GPIO_LCD->BRR = GPIO_DATA_MASK & ~(b << GPIO_DATA_SHIFT);
      GPIO_LCD->BSRR = GPIO_DATA_MASK & (b << GPIO_DATA_SHIFT);
      /* Assert /WR.  */
      GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_WR);
      // chopstx_usec_wait (1);
      /* Negate /WR.  */
      GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_WR);
    }

  /* Negate /CS.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_CS);
}

/* Issue command with N same data write.  */
void
lcd_command_filln (st7732_cmd_t cmd, uint8_t b, size_t n)
{
  lcd_command_common (cmd);

  /* Write loop.  */
  while (n-- > 0)
    {
      GPIO_LCD->BRR = GPIO_DATA_MASK & ~(b << GPIO_DATA_SHIFT);
      GPIO_LCD->BSRR = GPIO_DATA_MASK & (b << GPIO_DATA_SHIFT);
      /* Assert /WR.  */
      GPIO_LCD_CTRL->BRR = (1<< GPIO_LCD_WR);
      // chopstx_usec_wait (1);
      /* Negate /WR.  */
      GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_WR);
    }

  /* Negate /CS.  */
  GPIO_LCD_CTRL->BSRR = (1<< GPIO_LCD_CS);
}

static chopstx_mutex_t lcd_mtx;
static chopstx_cond_t lcd_cnd0;
static chopstx_cond_t lcd_cnd1;

/* Process for initializing ST7732.  */
static void *
lcd_initializer (void *arg __attribute__((unused)))
{
  uint8_t args[16];

  chopstx_mutex_lock (&lcd_mtx);
  chopstx_cond_wait (&lcd_cnd0, &lcd_mtx);
  chopstx_mutex_unlock (&lcd_mtx);

 /* Set GPIO_LCD to write mode.  */
  GPIO_LCD->CRH = 0x83333333;
  GPIO_LCD->CRL = 0x38888833;

  /* Set GPIO_LCD_CTRL IO mode.  */
  GPIO_LCD_CTRL->CRH = 0x88883888;
  GPIO_LCD_CTRL->CRL = 0x33333888;

  /* Restart ST7732.  */
  /* Hard reset.  */
  chopstx_usec_wait (100000);
  GPIO_LCD_CTRL->BRR = (1 << GPIO_LCD_RST);
  chopstx_usec_wait (100000);
  GPIO_LCD_CTRL->BSRR = (1 << GPIO_LCD_RST);
  chopstx_usec_wait (100000);

  /* Software reset.  */
  lcd_command_no (SWRESET);
  chopstx_usec_wait (150000);

  /* Sleep in.  */
  lcd_command_no (SLPIN);
  chopstx_usec_wait (100000);

  /* Sleep out.  */
  lcd_command_no (SLPOUT);
  chopstx_usec_wait (100000);

  /* Configure ST7732.  Set display mode, pixel mode, etc.  */
  /* FRMCTR1, 6, 3, 2  */
  args[0] = 0x06; args[1] = 0x03; args[2] = 0x02;
  lcd_command_writen (FRMCTR1, args, 3);
  /* INVCTR, 3  */
  args[0] = 0x03;
  lcd_command_writen (INVCTR, args, 1);
  /* DISSET5, 2, eh  */
  args[0] = 0x02; args[1] = 0x0e;
  lcd_command_writen (DISSET5, args, 2);
  /* DISPCTRL, 1ah  */
  args[0] = 0x1a;
  lcd_command_writen (DISPCTRL, args, 1);
  /* PWCTR1, 2, 0  */
  args[0] = 0x02; args[1] = 0x00;
  lcd_command_writen (PWCTR1, args, 2);
  /* PWCTR2, 5  */
  args[0] = 0x05;
  lcd_command_writen (PWCTR2, args, 1);
  /* PWCTR3, 2, 2  */
  args[0] = 0x02; args[1] = 0x02;
  lcd_command_writen (PWCTR3, args, 2);
  /* PWCTR4, 1, 2  */
  args[0] = 0x01; args[1] = 0x00;
  lcd_command_writen (PWCTR4, args, 2);
  /* PWCTR5, 1, 2  */
  args[0] = 0x01; args[1] = 0x00;
  lcd_command_writen (PWCTR5, args, 2);
  /* VMCTR1, 47h, 2ah  */
  args[0] = 0x47; args[1] = 0x2a;
  lcd_command_writen (VMCTR1, args, 2);
  /* OSCADJ, 4ch  */
  args[0] = 0x4c;
  lcd_command_writen (OSCADJ, args, 1);
  /* DEFADJ, 6  */
  args[0] = 0x06;
  lcd_command_writen (DEFADJ, args, 1);

  /* gamma adjust  */

  /* MADCTL, c0h  MY=1, MX=1  */
  args[0] = 0xc0;
  lcd_command_writen (MADCTL, args, 1);

  /* Set RA and CA.  */
  /* RASET, 0, 0, 0, 159  */
  args[0] = 0x00; args[1] = 0x00; args[2] = 0x00; args[3] = LCD_ROW-1;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, 0, 0, 127  */
  args[0] = 0x00; args[1] = 0x00; args[2] = 0x00; args[3] = LCD_COLUMN-1;
  lcd_command_writen (CASET, args, 4);

  /* 0x06: RGB 6-6-6-bit.  */
  args[0] = 0x06;
  lcd_command_writen (COLMOD, args, 1);

  args[0] = 0;
  lcd_command_writen (TEON, args, 1);

  lcd_command_no (DISPON);

  /* Wait 20ms.  */
  chopstx_usec_wait (20000);

  chopstx_mutex_lock (&lcd_mtx);
  chopstx_cond_signal (&lcd_cnd1);
  chopstx_mutex_unlock (&lcd_mtx);

  return NULL;
}

/* Plot a point with rgb color.  2 LSBs of rgb values will be ignored.  */
void
lcd_draw_point (int x, int y, int r, int g, int b)
{
  uint8_t args[4];

  /* Set RA and CA.  */
  /* RASET, 0, y, 0, y  */
  args[0] = 0x00; args[1] = y; args[2] = 0x00; args[3] = y;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, x, 0, x  */
  args[0] = 0x00; args[1] = x; args[2] = 0x00; args[3] = x;
  lcd_command_writen (CASET, args, 4);

  args[0] = r; args[1] = g; args[2] = b; 
  lcd_command_writen (RAMWR, args, 3);
}

static uint8_t hexfont5x8[16*5] = {
  0x7e, 0x89, 0x91, 0xa1, 0x7e, /* 0 */
  0x00, 0x41, 0xff, 0x01, 0x00, /* 1 */
  0x43, 0x85, 0x89, 0x91, 0x61, /* 2 */
  0x42, 0x81, 0x91, 0x91, 0x6e, /* 3 */
  0x18, 0x28, 0x48, 0xff, 0x08, /* 4 */
  0xf2, 0x91, 0x91, 0x91, 0x8e, /* 5 */
  0x1e, 0x29, 0x49, 0x89, 0x86, /* 6 */
  0x80, 0x8f, 0x90, 0xa0, 0xc0, /* 7 */
  0x6e, 0x91, 0x91, 0x91, 0x6e, /* 8 */
  0x70, 0x89, 0x89, 0x8a, 0x7c, /* 9 */
  0x7f, 0x88, 0x88, 0x88, 0x7f, /* A */
  0xff, 0x91, 0x91, 0x91, 0x6e, /* B */
  0x7e, 0x81, 0x81, 0x81, 0x42, /* C */
  0xff, 0x81, 0x81, 0x42, 0x3c, /* D */
  0xff, 0x91, 0x91, 0x91, 0x81, /* E */
  0xff, 0x90, 0x90, 0x90, 0x80, /* F */
};

/* Draw hex number with rgb color.  */
void
lcd_draw_hexfont5x8 (uint32_t hex, int x, int y, int r, int g, int b, int bg)
{
  int i, j;
  uint8_t *p;
  uint8_t args[5*8*3];

  p = &hexfont5x8[(hex & 0xf)*5];

  /* Set RA and CA.  */
  /* RASET, 0, y, 0, y+8-1  */
  args[0] = 0x00; args[1] = y; args[2] = 0x00; args[3] = y+7;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, x, 0, x+5-1  */
  args[0] = 0x00; args[1] = x; args[2] = 0x00; args[3] = x+4;
  lcd_command_writen (CASET, args, 4);

  for (i = 0; i < 5; i++)
    {
      uint8_t rb = *p++;
      for (j = 0; j < 8; j++)
	{
	  int k = (5*j+i)*3;
	  if (rb & (0x80 >> j))
	    {
	      args[k] = r; args[k+1] = g; args[k+2] = b;
	    }
	  else
	    {
	      args[k] = bg; args[k+1] = bg; args[k+2] = bg;
	    }
	}
    }

  lcd_command_writen (RAMWR, args, 5*8*3);
}

void
lcd_printhex (uint32_t hex, int x, int y, int r, int g, int b, int bg)
{
  int i;

  if (y < 0 || y >= LCD_ROW - 8)
    return;
  for (i = 7; i >= 0; i--)
    {
      lcd_draw_hexfont5x8 ((hex >> 4*i)&0xf, x, y, r, g, b, bg);
      x += 5;
      if (x >= LCD_COLUMN - 5)
	break;
    }
}

#define PRIO_LCD 3

extern uint8_t __process1_stack_base__, __process1_stack_size__;
const uint32_t __stackaddr_lcd = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_lcd = (size_t)&__process1_stack_size__;

/* Initialize LCD.  */
void
lcd_init (void)
{
  chopstx_mutex_init (&lcd_mtx);
  chopstx_cond_init (&lcd_cnd0);
  chopstx_cond_init (&lcd_cnd1);

  chopstx_create (PRIO_LCD, __stackaddr_lcd, __stacksize_lcd,
		  lcd_initializer, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&lcd_mtx);
  chopstx_cond_signal (&lcd_cnd0);
  chopstx_cond_wait (&lcd_cnd1, &lcd_mtx);
  chopstx_mutex_unlock (&lcd_mtx);
}
