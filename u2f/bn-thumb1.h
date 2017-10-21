/*
 * bn256_mul for ARM processours without umull instruction (Cortex-M0+)
 *
 * Some parts are generated and/or copied from TomsFastMath (public domain).
 * MULADD is adopted from "Shades of Elliptic Curve Cryptography on Embedded
 * Processors" by Erich Wenger, Thomas Unterluggauer and Mario Werner
 *
 * Some numbers:
 *
 *   this implementation takes 1.3120s to register / 0.6837s to authenticate
 *   uECC implementation takes 1.2960s to register / 0.6758s to authenticate
 * pure C implementation takes 1.6560s to register / 0.8558s to authenticate
 */

#define COMBA_START

#define COMBA_CLEAR \
   c0 = c1 = c2 = 0;

#define COMBA_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define COMBA_FINI

#define MULADD(i, j)                                       \
asm (                                                      \
   "ldr r1, %[_i]         \n\t"                            \
   "ldr r2, %[_j]         \n\t"                            \
   "uxth r6, r1           \n\t"                            \
   "uxth r7, r2           \n\t"                            \
   "lsr r1, r1, #16       \n\t"                            \
   "lsr r2, r2, #16       \n\t"                            \
                                                           \
   "mov r0, r6            \n\t"                            \
   "mul r0, r0, r7        \n\t"                            \
   "mul r6, r6, r2        \n\t"                            \
   "mul r2, r2, r1        \n\t"                            \
   "mul r1, r1, r7        \n\t"                            \
                                                           \
   "mov r7, #0            \n\t"                            \
   "add %0, %0, r0        \n\t"                            \
   "adc %1, %1, r2        \n\t"                            \
   "adc %2, %2, r7        \n\t"                            \
                                                           \
   "lsl r0, r6, #16       \n\t"                            \
   "lsr r2, r6, #16       \n\t"                            \
   "add %0, %0, r0        \n\t"                            \
   "adc %1, %1, r2        \n\t"                            \
   "adc %2, %2, r7        \n\t"                            \
                                                           \
   "lsl r0, r1, #16       \n\t"                            \
   "lsr r2, r1, #16       \n\t"                            \
   "add %0, %0, r0        \n\t"                            \
   "adc %1, %1, r2        \n\t"                            \
   "adc %2, %2, r7        \n\t"                            \
                                                           \
   :"=l"(c0), "=l"(c1), "=l"(c2)                           \
   : "0"(c0), "1"(c1), "2"(c2), [_i] "m" (i), [_j] "m" (j) \
   : "r0", "r1", "r2", "r6", "r7", "cc");

static inline void
bn256_mul_thumb1 (bn512 *X, const bn256 *A, const bn256 *B)
{
   uint32_t c0, c1, c2, at[16];

   memcpy(at, A->word, 8 * sizeof(uint32_t));
   memcpy(at+8, B->word, 8 * sizeof(uint32_t));
   COMBA_START;

   COMBA_CLEAR;
   /* 0 */
   MULADD(at[0], at[8]);
   COMBA_STORE(X->word[0]);
   /* 1 */
   COMBA_FORWARD;
   MULADD(at[0], at[9]);    MULADD(at[1], at[8]);
   COMBA_STORE(X->word[1]);
   /* 2 */
   COMBA_FORWARD;
   MULADD(at[0], at[10]);    MULADD(at[1], at[9]);    MULADD(at[2], at[8]);
   COMBA_STORE(X->word[2]);
   /* 3 */
   COMBA_FORWARD;
   MULADD(at[0], at[11]);    MULADD(at[1], at[10]);    MULADD(at[2], at[9]);    MULADD(at[3], at[8]);
   COMBA_STORE(X->word[3]);
   /* 4 */
   COMBA_FORWARD;
   MULADD(at[0], at[12]);    MULADD(at[1], at[11]);    MULADD(at[2], at[10]);    MULADD(at[3], at[9]);    MULADD(at[4], at[8]);
   COMBA_STORE(X->word[4]);
   /* 5 */
   COMBA_FORWARD;
   MULADD(at[0], at[13]);    MULADD(at[1], at[12]);    MULADD(at[2], at[11]);    MULADD(at[3], at[10]);    MULADD(at[4], at[9]);    MULADD(at[5], at[8]);
   COMBA_STORE(X->word[5]);
   /* 6 */
   COMBA_FORWARD;
   MULADD(at[0], at[14]);    MULADD(at[1], at[13]);    MULADD(at[2], at[12]);    MULADD(at[3], at[11]);    MULADD(at[4], at[10]);    MULADD(at[5], at[9]);    MULADD(at[6], at[8]);
   COMBA_STORE(X->word[6]);
   /* 7 */
   COMBA_FORWARD;
   MULADD(at[0], at[15]);    MULADD(at[1], at[14]);    MULADD(at[2], at[13]);    MULADD(at[3], at[12]);    MULADD(at[4], at[11]);    MULADD(at[5], at[10]);    MULADD(at[6], at[9]);    MULADD(at[7], at[8]);
   COMBA_STORE(X->word[7]);
   /* 8 */
   COMBA_FORWARD;
   MULADD(at[1], at[15]);    MULADD(at[2], at[14]);    MULADD(at[3], at[13]);    MULADD(at[4], at[12]);    MULADD(at[5], at[11]);    MULADD(at[6], at[10]);    MULADD(at[7], at[9]);
   COMBA_STORE(X->word[8]);
   /* 9 */
   COMBA_FORWARD;
   MULADD(at[2], at[15]);    MULADD(at[3], at[14]);    MULADD(at[4], at[13]);    MULADD(at[5], at[12]);    MULADD(at[6], at[11]);    MULADD(at[7], at[10]);
   COMBA_STORE(X->word[9]);
   /* 10 */
   COMBA_FORWARD;
   MULADD(at[3], at[15]);    MULADD(at[4], at[14]);    MULADD(at[5], at[13]);    MULADD(at[6], at[12]);    MULADD(at[7], at[11]);
   COMBA_STORE(X->word[10]);
   /* 11 */
   COMBA_FORWARD;
   MULADD(at[4], at[15]);    MULADD(at[5], at[14]);    MULADD(at[6], at[13]);    MULADD(at[7], at[12]);
   COMBA_STORE(X->word[11]);
   /* 12 */
   COMBA_FORWARD;
   MULADD(at[5], at[15]);    MULADD(at[6], at[14]);    MULADD(at[7], at[13]);
   COMBA_STORE(X->word[12]);
   /* 13 */
   COMBA_FORWARD;
   MULADD(at[6], at[15]);    MULADD(at[7], at[14]);
   COMBA_STORE(X->word[13]);
   /* 14 */
   COMBA_FORWARD;
   MULADD(at[7], at[15]);
   COMBA_STORE(X->word[14]);
   COMBA_STORE2(X->word[15]);
   COMBA_FINI;
}
