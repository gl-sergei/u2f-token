#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "board.h"

const uint8_t sys_version[8] = {
  3*2+2,	     /* bLength */
  0x03,		     /* bDescriptorType = USB_STRING_DESCRIPTOR_TYPE */
  /* sys version: "3.0" */
  '3', 0, '.', 0, '0', 0,
};

#if defined(USE_SYS3) || defined(USE_SYS_BOARD_ID)
const uint32_t sys_board_id = BOARD_ID;
const uint8_t sys_board_name[] = BOARD_NAME;
#endif

void
set_led (int on)
{
  puts (on ? "*": "");
}

static const char *flash_path;
static size_t flash_size;
static void * flash_addr;
static int flash_fd;

uintptr_t
flash_init (const char *f_name)
{
  int fd;
  struct stat sb;
  void *addr;

  fd = open (f_name, O_RDONLY);
  if (fd < 0)
    {
      perror ("flash_init: open");
      exit (1);
    }

  if (fstat (fd, &sb) < 0)
    {
      perror ("flash_init: fstat");
      exit (1);
    }

  addr = mmap (NULL, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED)
    {
      perror ("flash_init: mmap");
      exit (1);
    }

  if (close (fd) < 0)
    {
      perror ("flash_init: close");
      exit (1);
    }

  flash_path = f_name;
  flash_addr = addr;
  flash_size = sb.st_size;

  return (uintptr_t)addr;
}

void
flash_unlock (void)
{
  int fd;

  fd = open (flash_path, O_WRONLY);
  if (fd < 0)
    {
      perror ("flash_unlock: open");
      exit (1);
    }
  flash_fd = fd;
}

int
flash_program_halfword (uintptr_t addr, uint16_t data)
{
  off_t offset;
  char buf[2];

  fprintf (stderr, "flash_program_halfword: addr=%016lx, data=%04x\n", addr, data);
  offset = (off_t)(addr - (uintptr_t)flash_addr);
  if (offset < 0 || offset >= (off_t)flash_size)
    {
      perror ("flash_program_halfword");
      return 1;
    }

  offset = lseek (flash_fd, offset, SEEK_SET);
  if (offset == (off_t)-1)
    {
      perror ("flash_program_halfword");
      return 1;
    }
  buf[0] = (data & 0xff);
  buf[1] = (data >> 8);
  if (write (flash_fd, buf, 2) != 2)
    {
      perror ("flash_program_halfword");
      return 2;
    }
  return 0;
}

static const uint8_t erased[] = { [0 ... 1023 ] = 0xff };

int
flash_erase_page (uintptr_t addr)
{
  off_t offset;

  fprintf (stderr, "flash_erase_page: addr=%016lx\n", addr);

  offset = (off_t)(addr - (uintptr_t)flash_addr);
  if (offset < 0 || offset >= (off_t)flash_size)
    {
      perror ("flash_erase_page");
      return 1;
    }

  offset = lseek (flash_fd, offset, SEEK_SET);
  if (offset == (off_t)-1)
    {
      perror ("flash_erase_page");
      return 1;
    }

  if (write (flash_fd, erased, sizeof (erased)) != sizeof (erased))
    {
      perror ("flash_erase_page");
      return 2;
    }
  return 0;
}

int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  const uint8_t *p;

  if (p_start < (const uint8_t *)flash_addr
      || p_start + size > (const uint8_t *)flash_addr + flash_size)
    {
      perror ("flash_check_blank");
      return 0;
    }

  for (p = p_start; p < p_start + size; p++)
    if (*p != 0xff)
      return 0;

  return 1;
}

int
flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len)
{
  off_t offset;

  fprintf (stderr, "flash_write: addr=%016lx, %p, %zd\n", dst_addr, src, len);

  offset = (off_t)(dst_addr - (uintptr_t)flash_addr);
  if (offset < 0 || offset >= (off_t)flash_size)
    {
      perror ("flash_write");
      return 1;
    }

  offset = lseek (flash_fd, offset, SEEK_SET);
  if (offset == (off_t)-1)
    {
      perror ("flash_write");
      return 1;
    }

  if (write (flash_fd, src, len) != (ssize_t)len)
    {
      perror ("flash_write");
      return 2;
    }
  return 0;
}

int
flash_protect (void)
{
  fprintf (stderr, "flash_protect\n");
  return 0;
}

void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void))
{
  (void)entry;
  exit (1);
}

void
usb_lld_sys_init (void)
{
}

void
usb_lld_sys_shutdown (void)
{
}


void __attribute__((noreturn))
nvic_system_reset (void)
{
  exit (0);
}


void
clock_init (void)
{
}

void
gpio_init (void)
{
}
