#include <ucontext.h>
/*
 * The thread context: specific to GNU/Linux.
 *
 * We use the type ucontext_t, which includes all registers;
 * Note that signal mask is also included in ucontext_t.
 *
 */

typedef ucontext_t tcontext_t;
