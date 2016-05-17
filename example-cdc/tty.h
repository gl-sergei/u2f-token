struct tty;

struct tty *tty_open (void);
void tty_wait_configured (struct tty *tty);
void tty_wait_connection (struct tty *tty);
int tty_send (struct tty *tty, uint8_t *buf, int count);
int tty_recv (struct tty *tty, uint8_t *buf, uint32_t *timeout);
