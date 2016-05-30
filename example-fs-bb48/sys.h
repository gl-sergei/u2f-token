extern const uint8_t sys_version[8];
extern const uint32_t sys_board_id;
extern const char *const sys_board_name;

typedef void (*handler)(void);
extern handler sys_vector[16];

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
  (*sys_vector[0]) ();
}

static INLINE void
gpio_init (void)
{
  (*sys_vector[1]) ();
}

static inline void
set_led (int on)
{
  void (*func) (int) = (void (*)(int))sys_vector[2];

  return (*func) (on);
}

void crc32_init (unsigned int *);
void crc32_u8 (unsigned int *, unsigned char);
void crc32_u32 (unsigned int *, unsigned int);

int flash_erase_page (uint32_t addr);
int flash_program_word (uint32_t addr, uint32_t word);
