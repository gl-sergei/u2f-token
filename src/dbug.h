#ifndef __U2F_DBUG_H__

#define __U2F_DBUG_H__

void
dbg_send_command (int command, void *message);

void
dbg_print(const char *text);

#endif
