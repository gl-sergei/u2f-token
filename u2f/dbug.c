#include <stdint.h>
#include <string.h>

/* some debugging routines */
void
dbg_send_command (int command, void *message)
{
  asm ("mov r0, %[cmd];"
       "mov r1, %[msg];"
       "bkpt #0xAB"
       :
       : [cmd] "r" (command), [msg] "r" (message)
       : "r0", "r1", "memory");
}

void
dbg_print(const char *text)
{
  uint32_t msg[3];

  msg[0] = 2 /*stderr*/;
  msg[1] = (uint32_t) text;
  msg[2] = strlen(text);

  dbg_send_command (0x05, msg);
}
