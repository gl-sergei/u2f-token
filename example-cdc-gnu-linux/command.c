#include <string.h>
#include <stdint.h>
#include <chopstx.h>
#include "tty.h"
#include "config.h"
#ifdef ADC_SUPPORT
#include "adc.h"
static int adc_initialized = 0;
#endif
#include "board.h"
#include "sys.h"

struct command_table
{
  const char *name;
  void (*handler) (struct tty *tty, const char *line);
};

/*
 * Put a line (or lines) to TTY.
 * LINE should be terminated with newline.
 */
static void
put_line (struct tty *tty, const char *line)
{
  tty_send (tty, line, strlen (line));
}

static const char *help_string = 
  "mdb ADDR [COUNT];       memory display byte\r\n"
  "mwh ADDR VALUE [COUNT]; memory write halfword\r\n"
  "fes ADDR [COUNT];       flash erase sector\r\n"
  "fwh ADDR VALUE [COUNT]; flash write halfword\r\n"
#ifdef CRC32_SUPPORT
  "crc32 string;           CRC32 calc string\r\n"
#endif
#ifdef ADC_SUPPORT
  "adc;                    get 256-byte from ADC\r\n"
#endif
  "sysinfo;                system information\r\n"
  "help\r\n";

static char hexchar (uint8_t x)
{
  x &= 0x0f;
  if (x <= 0x09)
    return '0' + x;
  else if (x <= 0x0f)
    return 'a' + x - 10;
  else
    return '?';
}

#ifdef TOUCH_SUPPORT
static char *
compose_decimal (char *s, int value)
{
  uint32_t v;
  int col = 1000000000;
  int d;
  int digit_output = 0;

  if (value < 0)
    {
      *s++ = '-';
      v = 1 + ~((uint32_t)value);
    }
  else
    v = (uint32_t)value;

  while (col >= 10)
    {
      if (v >= (uint32_t)col)
	{
	  d = v / col;
	  v = v - d * col;
	  *s++ = d + '0';
	  digit_output = 1;
	}
      else if (digit_output)
	*s++ = '0';

      col = col / 10;
    }

  *s++ = v + '0';

  return s;
}
#endif


char *
compose_hex_ptr (char *s, uintptr_t v)
{
  s[0] = hexchar (v >> 60);
  s[1] = hexchar (v >> 56);
  s[2] = hexchar (v >> 52);
  s[3] = hexchar (v >> 48);
  s[4] = hexchar (v >> 44);
  s[5] = hexchar (v >> 40);
  s[6] = hexchar (v >> 36);
  s[7] = hexchar (v >> 32);
  s[8] = hexchar (v >> 28);
  s[9] = hexchar (v >> 24);
  s[10] = hexchar (v >> 20);
  s[11] = hexchar (v >> 16);
  s[12] = hexchar (v >> 12);
  s[13] = hexchar (v >> 8);
  s[14] = hexchar (v >> 4);
  s[15] = hexchar (v);
  return s+16;
}

static char *
compose_hex (char *s, uint32_t v)
{
  s[0] = hexchar (v >> 28);
  s[1] = hexchar (v >> 24);
  s[2] = hexchar (v >> 20);
  s[3] = hexchar (v >> 16);
  s[4] = hexchar (v >> 12);
  s[5] = hexchar (v >> 8);
  s[6] = hexchar (v >> 4);
  s[7] = hexchar (v);
  return s+8;
}

static char *
compose_hex_byte (char *s, uint8_t v)
{
  s[0] = hexchar (v >> 4);
  s[1] = hexchar (v);
  return s+2;
}

static const char *
get_hex (struct tty *tty, const char *s, uintptr_t *v_p)
{
  uintptr_t v = 0;
  char c;

  if (s[0] == '0' && s[1] == 'x')
    s = s + 2;
  while (1)
    {
      c = *s++;

      if (c == 0)
	{
	  s--;
	  break;
	}

      if (c == ' ')
	break;

      v = (v << 4);
      if (c >= '0' && c <= '9')
	v += (c - '0');
      else if (c >= 'a' && c <= 'f')
	v += (c - 'a') + 10;
      else if (c >= 'A' && c <= 'F')
	v += (c - 'A') + 10;
      else
	{
	  put_line (tty, "hex error\r\n");
	  return NULL;
	}
    }

  *v_p = v;
  return s;
}


#ifdef TOUCH_SUPPORT
#define TOUCH_VALUE_HIGH 100
#define TOUCH_VALUE_LOW  50
static void
cmd_button (struct tty *tty, const char *line)
{
  int i = 0;
  extern uint16_t touch_get (void);
  uint16_t v0 = 0;
  int touched = 0;

  (void)line;
  put_line (tty, "Please touch the bear.\r\n");

  while (i < 16)
    {
      uint16_t v = touch_get ();
      v0 = (v0 * 2 + v)/3;

      if (touched == 0 && v0 > TOUCH_VALUE_HIGH)
	{
	  tty_send (tty, "!", 1);
	  touched = 1;
	}
      else if (touched == 1 && v0 < TOUCH_VALUE_LOW)
	{
	  tty_send (tty, ".", 1);
	  touched = 0;
	  i++;
	}

      chopstx_usec_wait (10*1000);
    }

  tty_send (tty, "\r\n", 2);
}

static void
cmd_touch (struct tty *tty, const char *line)
{
  int i;
  extern uint16_t touch_get (void);

  (void)line;
  put_line (tty, "Please touch the bear.\r\n");

  for (i = 0; i < 20; i++)
    {
      uint16_t v;
      char output[8];
      char *s;

      chopstx_usec_wait (1000*1000);
      v = touch_get ();
      s = compose_decimal (output, v);
      *s++ = '\r';
      *s++ = '\n';
      tty_send (tty, output, s - output);
    }
}
#endif


static void
cmd_mdb (struct tty *tty, const char *line)
{
  int i;
  uintptr_t addr = 0;
  int count = 0;
  char c;
  const char *s = line;
  
  s = get_hex (tty, s, &addr);
  addr &= ~3;
  if (s == NULL)
    return;

  if (*s == 0)
    count = 1;
  else
    {
      while (1)
	{
	  c = *s++;

	  if (c == 0 || c == ' ')
	    break;

	  count = count * 10;
	  if (c >= '0' && c <= '9')
	    count += c - '0';
	  else
	    {
	      put_line (tty, "mdb error\r\n");
	      return;
	    }
	}
    }

  i = 0;
  while (i < count)
    {
      uint8_t v;
      char output[68];
      char *s;

      s = compose_hex_ptr (output, addr);
      *s++ = ':';
      *s++ = ' ';

      while (1)
	{
	  v = *(uint8_t *)addr;
	  s = compose_hex_byte (s, v);
	  i++;
	  addr += 1;
	  if (i >= count || (i % 16) == 0)
	    break;
	  *s++ = ' ';
	}

      *s++ = '\r';
      *s++ = '\n';
      tty_send (tty, output, s - output);
    }
}

static void
cmd_mwh (struct tty *tty, const char *line)
{
  (void)tty;
  (void)line;
  put_line (tty, "mwh not yet supported\r\n");
}

static void
cmd_fes (struct tty *tty, const char *line)
{
  int i;
  uintptr_t addr = 0;
  int count = 0;
  char c;
  const char *s = line;
  
  s = get_hex (tty, s, &addr);
  if (s == NULL)
    return;

  if (*s == 0)
    count = 1;
  else
    {
      while (1)
	{
	  c = *s++;

	  if (c == 0 || c == ' ')
	    break;

	  count = count * 10;
	  if (c >= '0' && c <= '9')
	    count += c - '0';
	  else
	    {
	      put_line (tty, "fww error\r\n");
	      return;
	    }
	}
    }

  for (i = 0; i < count; i++)
    {
      flash_erase_page (addr);
      addr += 1024;
    }
}

static void
cmd_fwh (struct tty *tty, const char *line)
{
  int i;
  uintptr_t addr = 0;
  uintptr_t d;
  uint16_t value = 0;
  int count = 0;
  char c;
  const char *s = line;
  
  s = get_hex (tty, s, &addr);
  if (s == NULL)
    return;

  if (*s == 0)
    return;

  s = get_hex (tty, s, &d);
  value = (uint16_t)d;
  if (s == NULL)
    return;

  if (*s == 0)
    count = 1;
  else
    {
      while (1)
	{
	  c = *s++;

	  if (c == 0 || c == ' ')
	    break;

	  count = count * 10;
	  if (c >= '0' && c <= '9')
	    count += c - '0';
	  else
	    {
	      put_line (tty, "fww error\r\n");
	      return;
	    }
	}
    }

  for (i = 0; i < count; i++)
    {
      flash_program_halfword (addr, value);
      addr += 4;
    }
}


#ifdef CRC32_SUPPORT
static unsigned int crc_value;

static void
cmd_crc32  (struct tty *tty, const char *line)
{
  uint32_t v;
  char string[10];
  char *s;

  crc32_init (&crc_value);
  while (*line)
    crc32_u8 (&crc_value, *line++);
  v = crc_value ^ 0xffffffff;

  s = compose_hex (string, v);
  *s++ = '\r';
  *s++ = '\n';
  tty_send (tty, string, sizeof (string));
}
#endif

#ifdef ADC_SUPPORT
static void
cmd_adc  (struct tty *tty, const char *line)
{
  int i;
  char output[73];
  char *s;

  (void)line;

  if (!adc_initialized)
    {
      if (adc_init ())
	{
	  put_line (tty, "adc_init error\r\n");
	  return;
	}
      else
	{
	  adc_start ();
	  adc_initialized = 1;
	}
    }

  adc_start_conversion (0, 64);
  adc_wait_completion ();

  i = 0;
  s = output;
  while (1)
    {
      s = compose_hex (s, adc_buf[i]);
      i++;
      if ((i % 8))
	*s++ = ' ';
      else
	{
	  *s++ = '\r';
	  *s++ = '\n';
	  tty_send (tty, output, s - output);
	  s = output;
	  if (i >= 64)
	    break;
	}
    }
}
#endif

static void
cmd_sysinfo (struct tty *tty, const char *line)
{
  char output[73];
  char *s;
  int i;

  (void)line;
  memcpy (output, "SYS version: ", 13);
  s = output + 13; 
  *s++ = sys_version[2];
  *s++ = sys_version[4];
  *s++ = sys_version[6];
  *s++ = '\r';
  *s++ = '\n';
  tty_send (tty, output, s - output);

  memcpy (output, "Board ID: ", 10);
  s = output + 10; 
  s = compose_hex (s, sys_board_id);
  *s++ = '\r';
  *s++ = '\n';
  tty_send (tty, output, s - output);

  memcpy (output, "Board name: ", 12);
  s = output + 12; 
  for (i = 0; i < (int)sizeof (output) - 2; i ++)
    if ((*s = sys_board_name[i]) == 0)
      break;
    else
      s++;

  *s++ = '\r';
  *s++ = '\n';
  tty_send (tty, output, s - output);
}


static void
cmd_help (struct tty *tty, const char *line)
{
  (void)line;
  put_line (tty, help_string);
}


struct command_table command_table[] = {
#ifdef TOUCH_SUPPORT
  { "button", cmd_button },
  { "touch", cmd_touch },
#endif
  { "mdb", cmd_mdb },
  { "mwh", cmd_mwh },
  { "fes", cmd_fes },
  { "fwh", cmd_fwh },
#ifdef CRC32_SUPPORT
  { "crc32", cmd_crc32 },
#endif
#ifdef ADC_SUPPORT
  { "adc", cmd_adc },
#endif
  { "sysinfo", cmd_sysinfo },
  { "help", cmd_help },
};

#define N_CMDS (int)(sizeof (command_table) / sizeof (struct command_table))


/*
 * Dispatch a command parsing LINE.
 * Line is NULL terminated with no newline.
 */
void
cmd_dispatch (struct tty *tty, const char *line)
{
  int i;
  const char *p;
  unsigned int n = 0;

  p = line;
  while (*p)
    {
      if (*p++ == ' ')
	break;
      n++;
    }

  for (i = 0; i < N_CMDS; i++)
    if (n == strlen (command_table[i].name)
	&& strncmp ((const char *)line, command_table[i].name, n) == 0)
      break;

  if (i != N_CMDS)
    (*command_table[i].handler) (tty, p);
  else
    {
      char crlf[] = { '\r', '\n' };

      put_line (tty, "No such command: ");
      tty_send (tty, line, n);
      tty_send (tty, crlf, sizeof (crlf));
    }
}
