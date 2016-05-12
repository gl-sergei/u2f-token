#define BUFSIZE 128
#define FLAG_CONNECTED   (1 << 0)
#define FLAG_SEND_AVAIL  (1 << 1)
#define FLAG_RECV_AVAIL  (1 << 2)

/*
 * Current implementation is synchronous and buffers are not yet used.
 */
struct stream {
  chopstx_mutex_t mtx;
  chopstx_cond_t cnd;
  int sending;
  unsigned int recv_len;
  uint8_t recv_buf[BUFSIZE];	/* Not yet used. */
  uint8_t buf_send[BUFSIZE];	/* Not yet used. */
  uint8_t cnt_send_head;	/* Not yet used. */
  uint8_t cnt_send_tail;	/* Not yet used. */
  uint8_t cnt_recv_head;	/* Not yet used. */
  uint8_t cnt_recv_tail;	/* Not yet used. */
  uint32_t flags;
};

struct stream *stream_open (void);
int stream_wait_connection (struct stream *st);
int stream_send (struct stream *st, uint8_t *buf, uint8_t count);
int stream_recv (struct stream *st, uint8_t *buf);
