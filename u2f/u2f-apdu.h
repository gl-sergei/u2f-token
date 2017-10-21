#ifndef __U2F_U2F_APDU_H__

#define __U2F_U2F_APDU_H__

void
u2f_apdu_init (void);

int
u2f_apdu_command_do (uint8_t *apdu, uint8_t len,
                     uint8_t *resp, uint32_t *resp_len);

#endif
