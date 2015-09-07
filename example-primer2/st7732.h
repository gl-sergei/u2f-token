/* ST7732 LCD driver chip command byte.
  command_name = value  read_n_bytes:write_n_bytes: simple description  */
enum st7732_cmd {
  NOP = 0x00,		/* 0:0: No Operatin */
  SWRESET = 0x01,	/* 0:0: Software reset */
  RDDID = 0x04,		/* 0:3: Read Display ID */
  RDRST = 0x09,		/* 0:4: Read Display Status */
  RDDPM = 0x0a,		/* 0:1: Read Display Power Mode */
  RDD_MADCTL = 0x0b,	/* 0:1: Read Display MADCTL */
  RDD_COLMOD = 0x0c,	/* 0:1: Read Display Pixel Format */
  RDDIM = 0x0d,		/* 0:1: Read Display Image Mode */
  RDDSM = 0x0e,		/* 0:1: Read Display Signal Mode */
  RDDSDR = 0x0f,	/* 0:1: Read Display Self-diagnostic result */
  SLPIN = 0x10,		/* 0:0: Sleep in & booster off */
  SLPOUT = 0x11,	/* 0:0: Sleep out & booster on */
  PTLON = 0x12,		/* 0:0: Pertial mode on */
  NORON = 0x13,		/* 0:0: Normal mode on (Pertial off) */
  INVOFF = 0x20,	/* 0:0: Display inversion off */
  INVON = 0x21,		/* 0:0: Display inversion on */
  GAMSET = 0x26,	/* 1:0: Gamma curve select */
  DISPOFF = 0x28,	/* 0:0: Display off */
  DISPON = 0x29,	/* 0:0: Display on */
  CASET = 0x2a,		/* 4:0: Column address set */
  RASET = 0x2b,		/* 4:0: Raw address set */
  RAMWR = 0x2c,		/* 1:0: Memory write */
  RAMRD = 0x2e,		/* 0:1: Memory read */
  PTLAR = 0x30,		/* 4:0: Partial start/end address set */
  SCRLAR = 0x33,	/* 6:0: Scroll area set */
  TEOFF = 0x34,		/* 0:0: Tearing effect line off */
  TEON = 0x35,		/* 1:0: Tearing effect mode set & on */
  MADCTL = 0x36,	/* 1:0: Memory data access control */
  VSCSAD = 0x37,	/* 2:0: Scroll start address of RAM */
  IDMOFF = 0x38,	/* 0:0: Idle mode off */
  IDMON = 0x39,		/* 0:0: Idle mode on */
  COLMOD = 0x3a,	/* 1:0: Interface pixel format */
  RDID1 = 0xda,		/* 0:1: Read ID1 */
  RDID2 = 0xdb,		/* 0:1: Read ID2 */
  RDID3 = 0xdc,		/* 0:1: Read ID3 */
  RGBCTR = 0xb0,	/* 1:0: Set RGB signal control */
  FRMCTR1 = 0xb1,	/* 3:0: In normal mode */
  FRMCTR2 = 0xb2,	/* 3:0: In Idel mode (8-colors) */
  FRMCTR3 = 0xb3,	/* 6:0: In partial mode + Full colors */
  INVCTR = 0xb4,	/* 1:0: Display inversion control */
  RGB_BPCTR = 0xb5,	/* 4:0: RGB I/F Blanking porch setting */
  DISSET5 = 0xb6,	/* 2:0: Display function setting */
  PWCTR1 = 0xc0,	/* 2:0: Power control setting */
  PWCTR2 = 0xc1,	/* 1:0: Power control setting */
  PWCTR3 = 0xc2,	/* 2:0: Power control setting (Full colors) */
  PWCTR4 = 0xc3,	/* 2:0: Power control setting (8-colors) */
  PWCTR5 = 0xc4,	/* 2:0: Power control setting (In partial mode) */
  VMCTR1 = 0xc5,	/* 2:0: VCOM control */
  VMOFCTR = 0xc6,	/* 1:0: VCOM offset control */
  WRID2 = 0xd1,		/* 1:0: Write ID2 value to NV */
  WRID3 = 0xd2,		/* 1:0: Write ID3 value to NV */
  RDID4 = 0xd3,		/* 0:4: IC Vender code */
  NVCTR1 = 0xd9,	/* 0:1:no-fummy NVM control status */
  NVCTR2 = 0xde,	/* 3:0: NVM read command (aa, 0f, a5) */
  NVCTR3 = 0xdf,	/* 3:0: NVM write command (55, f0, 5a) */
  GAMCTRP1 = 0xe0,	/* 13:0: Set Gamma correction + */
  GAMCTRN1 = 0xe1,	/* 13:0: Set Gamma correction - */
  AUTO_CTRL = 0xf1,	/* 1:0: NVM write function ON/OFF */
  OSCADJ = 0xf2,	/* 1:0: Osillator frequency setting */
  DISPCTRL = 0xf5,	/* 1:0: Display function control */
  DEFADJ = 0xf6,	/* 1:0: Default mode setting */
};

typedef enum st7732_cmd st7732_cmd_t;

extern void lcd_command_no (st7732_cmd_t cmd);
extern void lcd_command_readn (st7732_cmd_t cmd, uint8_t *p, size_t n);
extern void lcd_command_writen (st7732_cmd_t cmd, uint8_t *p, size_t n);
extern void lcd_command_filln (st7732_cmd_t cmd, uint8_t b, size_t n);
extern void lcd_init (void);

extern void lcd_draw_point (int x, int y, int r, int g, int b);
extern void lcd_draw_hexfont5x8 (uint32_t hex, int x, int y, int r, int g,
				 int b, int bg);
extern void lcd_printhex (uint32_t hex, int x, int y, int r, int g, int b,
			  int bg);

#define LCD_COLUMN	128
#define LCD_ROW		160
#define BYTES_PER_PIXEL	3
