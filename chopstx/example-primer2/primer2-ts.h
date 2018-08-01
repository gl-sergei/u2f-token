extern void adc3_init (void);
extern void adc3_start (void);
extern void adc3_conversion (uint32_t *result);
extern void adc3_stop (void);

extern int ts_pushed (uint32_t u);
extern void ts_conversion (uint32_t a[], int r[]);
extern int ts_adjust (int r[], int cord[]);
