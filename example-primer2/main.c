#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include "neug.h"
#include "sys.h" /* for set_led */
#include "stm32f103.h"
#include "adc.h"
#include "st7732.h"
#include "primer2-switches.h"
#include "primer2-ts.h"
#include "board.h"

#ifdef TEST_DISPLAY_LOGO
static uint8_t buf[LCD_COLUMN*LCD_ROW*BYTES_PER_PIXEL] = {
#include "chopstx-logo.data"
};

void
lcd_logo (void)
{
  uint8_t args[4];

  /* Set RA and CA.  */
  /* RASET, 0, 0, 0, 159  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_ROW-1;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, 0, 0, 127  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_COLUMN-1;
  lcd_command_writen (CASET, args, 4);

  /* Write logo.  */
  lcd_command_writen (RAMWR, buf, LCD_COLUMN*LCD_ROW*BYTES_PER_PIXEL);
}
#endif

/* Table of 32*(cos, sin) for range 0 to pi/2 with step pi/256.  */
static uint8_t ctable[128*2] = {
32, 0, 31, 0, 31, 0, 31, 1, 31, 1, 31, 1, 31, 2, 31, 2,
31, 3, 31, 3, 31, 3, 31, 4, 31, 4, 31, 5, 31, 5, 31, 5,
31, 6, 31, 6, 31, 7, 31, 7, 31, 7, 30, 8, 30, 8, 30, 8,
30, 9, 30, 9, 30, 10, 30, 10, 30, 10, 29, 11, 29, 11, 29, 11,
29, 12, 29, 12, 29, 12, 29, 13, 28, 13, 28, 14, 28, 14, 28, 14,
28, 15, 28, 15, 27, 15, 27, 16, 27, 16, 27, 16, 27, 17, 26, 17,
26, 17, 26, 18, 26, 18, 25, 18, 25, 19, 25, 19, 25, 19, 24, 19,
24, 20, 24, 20, 24, 20, 23, 21, 23, 21, 23, 21, 23, 22, 22, 22,
22, 22, 22, 22, 22, 23, 21, 23, 21, 23, 21, 23, 20, 24, 20, 24,
20, 24, 19, 24, 19, 25, 19, 25, 19, 25, 18, 25, 18, 26, 18, 26,
17, 26, 17, 26, 17, 27, 16, 27, 16, 27, 16, 27, 15, 27, 15, 28,
15, 28, 14, 28, 14, 28, 14, 28, 13, 28, 13, 29, 12, 29, 12, 29,
12, 29, 11, 29, 11, 29, 11, 29, 10, 30, 10, 30, 10, 30, 9, 30,
9, 30, 8, 30, 8, 30, 8, 30, 7, 31, 7, 31, 7, 31, 6, 31,
6, 31, 5, 31, 5, 31, 5, 31, 4, 31, 4, 31, 3, 31, 3, 31,
3, 31, 2, 31, 2, 31, 1, 31, 1, 31, 1, 31, 0, 31, 0, 31,
};

#ifdef TEST_LCD_CIRCLE
void
lcd_circle (void)
{
  int i, j;
  uint8_t *p;
  int x, y;

  /* Clear display.  */
  /* Set RA and CA.  */
  /* RASET, 0, 0, 0, 159  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_ROW-1;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, 0, 0, 127  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_COLUMN-1;
  lcd_command_writen (CASET, args, 4);

  lcd_command_writen (RAMWR, 0, LCD_COLUMN*LCD_ROW*BYTES_PER_PIXEL);

  /* Draw a circle.  */
  for (i = 0; i < 128; i++)
    {
      x = 64 + ctable[2*i];
      y = 80 + ctable[2*i+1];
      lcd_draw_point (x, y, 0xfc, 0xfc, 0xfc);
    }
  for (i = 0; i < 128; i++)
    {
      x = 64 - ctable[2*i+1];
      y = 80 + ctable[2*i];
      lcd_draw_point (x, y, 0xfc, 0, 0xfc);
    }
  for (i = 0; i < 128; i++)
    {
      x = 64 - ctable[2*i];
      y = 80 - ctable[2*i+1];
      lcd_draw_point (x, y, 0, 0xfc, 0xfc);
    }
  for (i = 0; i < 128; i++)
    {
      x = 64 + ctable[2*i+1];
      y = 80 - ctable[2*i];
      lcd_draw_point (x, y, 0xfc, 0xfc, 0);
    }
}
#endif

#define RANDOM_BYTES_LENGTH 64
static uint32_t random_word[RANDOM_BYTES_LENGTH/sizeof (uint32_t)];

int
main (int argc, const char *argv[])
{
  int count;
  int vx, vy;
  int r, g, b;
  uint8_t args[4];

  (void)argc;
  (void)argv;

  set_led (1);
  set_backlight (1);

  adc_init ();
  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  lcd_init ();

#ifdef TEST_LCD_LOGO
  lcd_logo ();

  while (! joystick ())
    chopstx_usec_wait (500*1000);
#endif

  /* Set RA and CA.  */
  /* RASET, 0, 0, 0, 159  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_ROW-1;
  lcd_command_writen (RASET, args, 4);
  /* CASET, 0, 0, 0, 127  */
  args[0] = 0x00; args[1] = 0; args[2] = 0x00; args[3] = LCD_COLUMN-1;
  lcd_command_writen (CASET, args, 4);

  /* Fill display.  */
  lcd_command_filln (RAMWR, 0xfc, LCD_COLUMN*LCD_ROW*BYTES_PER_PIXEL);

  vx = (LCD_COLUMN/2) << 5;
  vy = (LCD_ROW/2) << 5;
  r = g = b = 0;
#if 1
  adc3_init ();
  adc3_start ();

  count = 0;
  while (1)
    {
      uint32_t resv[4];

      adc3_conversion (resv);
      if (ts_pushed (resv[2]))
	{
	  int reg[3], point[2];

	  ts_conversion (resv, reg);
#if 0
	  lcd_printhex (reg[0], 5,  8, 0x00, 0x00, 0xfc, 0xfc);
	  lcd_printhex (reg[1], 5,  18, 0x00, 0x00, 0xfc, 0xfc);
	  lcd_printhex (reg[2], 5,  28, 0x00, 0x00, 0xfc, 0xfc);
#endif
	  if (!ts_adjust (reg, point))
	    {
	      chopstx_usec_wait (50*1000);
	      continue;
	    }

	  lcd_draw_point (point[0], point[1], r, g, b);
	}
      else
	ts_adjust (NULL, NULL);

      chopstx_usec_wait (50*1000);
      count++;
      if ((count/10) & 1)
	set_led (0);
      else
	set_led (1);
      if (pbutton())
	break;
    }

  adc3_stop ();
#endif

  count = 0;
  while (1)
    {
      int jo;
      uint32_t th = neug_get (NEUG_KICK_FILLING) & 0x1ff;

      /* Get random point on a circle with the radius of 32 and walk
	 towards it.  */
      if (th < 128)
	{
	  vx += ctable[2*th];
	  vy += ctable[2*th+1];
	}
      else if (th < 256)
	{
	  vx -= ctable[2*(th & 0x7f)+1];
	  vy += ctable[2*(th & 0x7f)];
	}
      else if (th < 384)
	{
	  vx -= ctable[2*(th & 0x7f)];
	  vy -= ctable[2*(th & 0x7f)+1];
	}
      else
	{
	  vx += ctable[2*(th & 0x7f)+1];
	  vy -= ctable[2*(th & 0x7f)];
	}

      if (vx < 0)
	vx += (LCD_COLUMN << 5);
      if (vy < 0)
	vy += (LCD_ROW << 5);

      /* Change draw color with joystick.  */
      jo = joystick ();
      if (JOYSTICK_L (jo))
	r = 0xfc;
      if (JOYSTICK_R (jo))
	g = 0xfc;
      if (JOYSTICK_U (jo))
	b = 0xfc;
      if (JOYSTICK_D (jo))
	r = g = b = 0;

      /* 2-dim random walk on torus.  */
      lcd_draw_point ((vx>>5)%LCD_COLUMN, (vy>>5)%LCD_ROW, r, g, b);
      chopstx_usec_wait (10*1000);

      if (pbutton ())
	count++;
      /* Shutdown when p-button is held down for 5 sec.  */
      if (count > 500)
	{
	  set_led (0);
	  shutdown ();
	}
      else
	{
	  /* Disable backlight when p-button is held down for 3 sec.  */
	  if (count > 300)
	    set_backlight (0);

	  /* Blink led when p-button is held.  */
	  if ((count/50) & 1)
	    set_led (0);
	  else
	    set_led (1);
	}
#if 1
      lcd_printhex (count, 5,  8, 0x00, 0x00, 0xfc, 0xfc);
#endif

    }

  return 0;
}
