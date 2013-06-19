typedef uint32_t eventmask_t;

struct eventflag {
  chopstx_t owner;
  uint32_t wait_usec;
  eventmask_t flag;
  chopstx_mutex_t mutex;
};

void eventflag_init (struct eventflag *ev, chopstx_t owner);
eventmask_t eventflag_wait_timeout (struct eventflag *ev, uint32_t usec);
void eventflag_signal (struct eventflag *ev, eventmask_t m);
