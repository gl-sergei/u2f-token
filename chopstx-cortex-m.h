/*
 * The thread context: specific to ARM Cortex-M0/M3.
 *
 * In this structure, it's only partial information; Other part of the
 * context is on the stack.
 *
 */
struct tcontext {
  uint32_t reg[9];	   /* r4, r5, r6, r7, r8, r9, r10, r11, r13(sp) */
};

typedef struct tcontext tcontext_t;
