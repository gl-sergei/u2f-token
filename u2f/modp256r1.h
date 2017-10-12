extern const bn256 p256r1;
#define P256R1 (&p256r1)

void modp256r1_add (bn256 *X, const bn256 *A, const bn256 *B);
void modp256r1_sub (bn256 *X, const bn256 *A, const bn256 *B);
void modp256r1_reduce (bn256 *X, const bn512 *A);
void modp256r1_mul (bn256 *X, const bn256 *A, const bn256 *B);
void modp256r1_sqr (bn256 *X, const bn256 *A);
void modp256r1_shift (bn256 *X, const bn256 *A, int shift);
