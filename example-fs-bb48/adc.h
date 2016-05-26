int adc_init (void);
void adc_start (void);
void adc_stop (void);

extern uint32_t adc_buf[64];

void adc_start_conversion (int offset, int count);
int adc_wait_completion (void);
