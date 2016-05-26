#include <string.h>
#include <stdint.h>
#include <chopstx.h>
#include "tty.h"
#include "config.h"
#ifdef ADC_SUPPORT
#include "adc.h"
static int adc_initialized = 0;
#endif

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
  tty_send (tty, (uint8_t *)line, strlen (line));
}

static const char *help_string = 
  "mdw ADDR [COUNT]\r\n"
  "mww ADDR VALUE [COUNT]\r\n"
#ifdef CRC32_SUPPORT
  "crc32 string\r\n"
#endif
#ifdef ADC_SUPPORT
  "adc\r\n"
#endif
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

#ifdef ENABLE_DECIMAL_OUTPUT
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

static void
cmd_mdw (struct tty *tty, const char *line)
{
  int i;
  uint32_t addr = 0;
  int count = 0;
  char c;

  if (line[0] == '0' && line[1] == 'x')
    line = line + 2;
  while (1)
    {
      c = *line++;

      if (c == 0)
	{
	  count = 1;
	  break;
	}

      if (c == ' ')
	break;

      addr = (addr << 4);
      if (c >= '0' && c <= '9')
	addr += (c - '0');
      else if (c >= 'a' && c <= 'f')
	addr += (c - 'a') + 10;
      else if (c >= 'A' && c <= 'F')
	addr += (c - 'A') + 10;
      else
	{
	  put_line (tty, "mdw error\r\n");
	  return;
	}
    }

  addr &= ~3;

  if (count == 0)
    {
      while (1)
	{
	  c = *line++;

	  if (c == 0 || c == ' ')
	    break;

	  count = count * 10;
	  if (c >= '0' && c <= '9')
	    count += c - '0';
	  else
	    {
	      put_line (tty, "mdw error\r\n");
	      return;
	    }
	}
    }

  i = 0;
  while (i < count)
    {
      uint32_t v;
      char output[48];
      char *s;

      s = compose_hex (output, addr);
      *s++ = ':';
      *s++ = ' ';

      while (1)
	{
	  v = *(uint32_t *)addr;
	  s = compose_hex (s, v);
	  i++;
	  addr += 4;
	  if (i >= count || (i % 4) == 0)
	    break;
	  *s++ = ' ';
	}

      *s++ = '\r';
      *s++ = '\n';
      tty_send (tty, (uint8_t *)output, s - output);
    }
}

static void
cmd_mww (struct tty *tty, const char *line)
{
  (void)tty;
  (void)line;
  put_line (tty, "mww not yet supported\r\n");
}


#ifdef CRC32_SUPPORT
#include "crc32.h"

static void
cmd_crc32  (struct tty *tty, const char *line)
{
  uint32_t v;
  char string[10];
  char *s;

  crc32_init ();
  while (*line)
    crc32_u8 (*line++);
  v = crc32_value () ^ 0xffffffff;

  s = compose_hex (string, v);
  *s++ = '\r';
  *s++ = '\n';
  tty_send (tty, (uint8_t *)string, sizeof (string));
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
	  tty_send (tty, (uint8_t *)output, s - output);
	  s = output;
	  if (i >= 64)
	    break;
	}
    }
}
#endif

static void
cmd_help (struct tty *tty, const char *line)
{
  (void)line;
  put_line (tty, help_string);
}


struct command_table command_table[] = {
  { "mdw", cmd_mdw },
  { "mww", cmd_mww },
#ifdef CRC32_SUPPORT
  { "crc32", cmd_crc32 },
#endif
#ifdef ADC_SUPPORT
  { "adc", cmd_adc },
#endif
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
      uint8_t crlf[] = { '\r', '\n' };

      put_line (tty, "No such command: ");
      tty_send (tty, (const uint8_t *)line, n);
      tty_send (tty, crlf, sizeof (crlf));
    }
}
