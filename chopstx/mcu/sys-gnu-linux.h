extern const uint8_t sys_version[8];
#if defined(USE_SYS3) || defined(USE_SYS_BOARD_ID)
extern const uint32_t sys_board_id;
extern const uint8_t sys_board_name[];
# define SYS_BOARD_ID sys_board_id
#else
# define SYS_BOARD_ID BOARD_ID
#endif

static inline const uint8_t *
unique_device_id (void)
{
  /*
   * STM32F103 has 96-bit unique device identifier.
   * This routine mimics that.
   */

  static const uint8_t id[] = { /* My RSA fingerprint */
    0x12, 0x41, 0x24, 0xBD, 0x3B, 0x48, 0x62, 0xAF,
    0x7A, 0x0A, 0x42, 0xF1, 0x00, 0xB4, 0x5E, 0xBD,
    0x4C, 0xA7, 0xBA, 0xBE
  };
  
  return id;
}

void set_led (int on);

uintptr_t flash_init (const char *f_name);
void flash_unlock (void);
int flash_program_halfword (uintptr_t addr, uint16_t data);
int flash_erase_page (uintptr_t addr);
int flash_check_blank (const uint8_t *p_start, size_t size);
int flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len);
int flash_protect (void);
void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void));

void usb_lld_sys_init (void);
void usb_lld_sys_shutdown (void);

void __attribute__((noreturn))
nvic_system_reset (void);

void clock_init (void);
void gpio_init (void);
