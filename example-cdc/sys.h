#define BOARD_ID_CQ_STARM          0xc5480875
#define BOARD_ID_FST_01_00         0x613870a9
#define BOARD_ID_FST_01            0x696886af
#define BOARD_ID_MAPLE_MINI        0x7a445272
#define BOARD_ID_OLIMEX_STM32_H103 0xf92bb594
#define BOARD_ID_STBEE_MINI        0x1f341961
#define BOARD_ID_STBEE             0x945c37e8
#define BOARD_ID_STM32_PRIMER2     0x21e5798d
#define BOARD_ID_STM8S_DISCOVERY   0x2f0976bb
#define BOARD_ID_ST_DONGLE         0x2cd4e471
#define BOARD_ID_ST_NUCLEO_F103    0x9b87c16d

extern const uint8_t sys_version[8];
extern const uint32_t sys_board_id;
extern const uint8_t sys_board_name[];

typedef void (*handler)(void);
extern handler vector[16];

static inline const uint8_t *
unique_device_id (void)
{
  /* STM32F103 has 96-bit unique device identifier */
  const uint8_t *addr = (const uint8_t *)0x1ffff7e8;

  return addr;
}

static inline void
set_led (int on)
{
  void (*func) (int) = (void (*)(int))vector[2];

  return (*func) (on);
}

static inline void
flash_unlock (void)
{
  (*vector[3]) ();
}

static inline int
flash_program_halfword (uint32_t addr, uint16_t data)
{
  int (*func) (uint32_t, uint16_t) = (int (*)(uint32_t, uint16_t))vector[4];

  return (*func) (addr, data);
}

static inline int
flash_erase_page (uint32_t addr)
{
  int (*func) (uint32_t) = (int (*)(uint32_t))vector[5];

  return (*func) (addr);
}

static inline int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  int (*func) (const uint8_t *, int) = (int (*)(const uint8_t *, int))vector[6];

  return (*func) (p_start, size);
}

static inline int
flash_write (uint32_t dst_addr, const uint8_t *src, size_t len)
{
  int (*func) (uint32_t, const uint8_t *, size_t)
    = (int (*)(uint32_t, const uint8_t *, size_t))vector[7];

  return (*func) (dst_addr, src, len);
}

static inline int
flash_protect (void)
{
  int (*func) (void) = (int (*)(void))vector[8];

  return (*func) ();
}

static inline void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void))
{
  void (*func) (void (*)(void)) = (void (*)(void (*)(void)))vector[9];

  (*func) (entry);
  for (;;);
}

static inline void
usb_lld_sys_init (void)
{
  (*vector[10]) ();
}

static inline void
usb_lld_sys_shutdown (void)
{
  (*vector[11]) ();
}

static inline void
nvic_system_reset (void)
{
  (*vector[12]) ();
}

/*
 * Users can override INLINE by 'attribute((used))' to have an
 * implementation defined.
 */
#if !defined(INLINE)
#define INLINE __inline__
#endif

static INLINE void
clock_init (void)
{
  (*vector[13]) ();
}

static INLINE void
gpio_init (void)
{
  (*vector[14]) ();
}
