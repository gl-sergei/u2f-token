/*
 * dbug.c - debugging routines
 *
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware for STM32
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

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
