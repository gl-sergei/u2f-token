const unsigned int *const crc32_table= (const unsigned int *)0x00000480;

void
crc32_init (unsigned int *p)
{
  *p = 0xffffffff;
}

static void
crc32_u8 (unsigned int *p, unsigned char v)
{
  *p = crc32_table[(*p & 0xff) ^ v] ^ (*p >> 8);
}

void
crc32_u32 (unsigned int *p, unsigned int u)
{
  crc32_u8 (p, u & 0xff);
  crc32_u8 (p, (u >> 8)& 0xff);
  crc32_u8 (p, (u >> 16)& 0xff);
  crc32_u8 (p, (u >> 24)& 0xff);
}
