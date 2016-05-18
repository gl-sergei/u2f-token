typedef uint32_t eventmask_t;

struct eventflag {
  eventmask_t flags;
  chopstx_mutex_t mutex;
  chopstx_cond_t cond;
};

void eventflag_init (struct eventflag *ev);
eventmask_t eventflag_wait (struct eventflag *ev);
eventmask_t eventflag_wait_timeout (struct eventflag *ev, uint32_t usec);
void eventflag_signal (struct eventflag *ev, eventmask_t m);
