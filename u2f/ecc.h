
int ecdsa_sign_p256r1 (const uint8_t *hash, uint8_t *output,
           const uint8_t *key_data);
uint8_t *ecc_compute_public_p256r1 (const uint8_t *key_data, uint8_t *public);
int ecc_check_secret_p256r1 (const uint8_t *d0, uint8_t *d1);
int ecdh_decrypt_p256r1 (const uint8_t *input, uint8_t *output,
       const uint8_t *key_data);
