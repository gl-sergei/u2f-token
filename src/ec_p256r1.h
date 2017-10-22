int compute_kP_p256r1 (ac *X, const bn256 *K, const ac *P);
int compute_kG_p256r1 (ac *X, const bn256 *K);
void ecdsa_p256r1 (bn256 *r, bn256 *s, const bn256 *z, const bn256 *d);
int check_secret_p256r1 (const bn256 *q, bn256 *d1);
