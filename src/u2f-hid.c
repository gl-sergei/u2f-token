/*
 * u2f-hid.c - U2F HID protocol
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
#include <chopstx.h>

#include "board.h"
#include "sys.h"
#include "usb-hid.h"
#include "u2f-hid.h"
#include "u2f-apdu.h"

// Size of HID reports 

#define HID_RPT_SIZE            64      // Default size of raw HID report

// Frame layout - command- and continuation frames

#define CID_BROADCAST           0xffffffff // Broadcast channel id

#define TYPE_MASK               0x80    // Frame type mask 
#define TYPE_INIT               0x80    // Initial frame identifier
#define TYPE_CONT               0x00    // Continuation frame identifier

typedef struct {
  uint32_t cid;                        // Channel identifier
  union {
    uint8_t type;                      // Frame type - b7 defines type
    struct {
      uint8_t cmd;                     // Command - b7 set
      uint8_t bcnth;                   // Message byte count - high part
      uint8_t bcntl;                   // Message byte count - low part
      uint8_t data[HID_RPT_SIZE - 7];  // Data payload
    } init;
    struct {
      uint8_t seq;                     // Sequence number - b7 cleared
      uint8_t data[HID_RPT_SIZE - 5];  // Data payload
    } cont;
  };
} __attribute__ ((packed)) U2FHID_FRAME;

#define FRAME_TYPE(f) ((f).type & TYPE_MASK)
#define FRAME_CMD(f)  ((f).init.cmd & ~TYPE_MASK)
#define MSG_LEN(f)    ((f).init.bcnth*256 + (f).init.bcntl)
#define FRAME_SEQ(f)  ((f).cont.seq & ~TYPE_MASK)

#define SET_MSG_LEN(f, len) { \
                              (f).init.bcnth = (uint8_t) ((len) >> 8); \
                              (f).init.bcntl = (uint8_t) (len); \
                            }

// HID usage- and usage-page definitions

#define FIDO_USAGE_PAGE         0xf1d0  // FIDO alliance HID usage page
#define FIDO_USAGE_U2FHID       0x01    // U2FHID usage for top-level collection
#define FIDO_USAGE_DATA_IN      0x20    // Raw IN data report
#define FIDO_USAGE_DATA_OUT     0x21    // Raw OUT data report
        
// General constants    

#define U2FHID_IF_VERSION       2       // Current interface implementation version
#define U2FHID_TRANS_TIMEOUT    3000    // Default message timeout in ms

// U2FHID native commands

#define U2FHID_PING         (TYPE_INIT | 0x01)  // Echo data through local processor only
#define U2FHID_MSG          (TYPE_INIT | 0x03)  // Send U2F message frame
#define U2FHID_LOCK         (TYPE_INIT | 0x04)  // Send lock channel command
#define U2FHID_INIT         (TYPE_INIT | 0x06)  // Channel initialization
#define U2FHID_WINK         (TYPE_INIT | 0x08)  // Send device identification wink
#define U2FHID_SYNC         (TYPE_INIT | 0x3c)  // Protocol resync command
#define U2FHID_ERROR        (TYPE_INIT | 0x3f)  // Error response

#define U2FHID_VENDOR_FIRST (TYPE_INIT | 0x40)  // First vendor defined command
#define U2FHID_VENDOR_LAST  (TYPE_INIT | 0x7f)  // Last vendor defined command
    
// U2FHID_INIT command defines

#define INIT_NONCE_SIZE         8       // Size of channel initialization challenge
#define CAPFLAG_WINK            0x01    // Device supports WINK command
#define CAPFLAG_LOCK            0x02    // Device supports LOCK command

typedef struct {
  uint8_t nonce[INIT_NONCE_SIZE];       // Client application nonce
} __attribute__ ((packed)) U2FHID_INIT_REQ;

typedef struct {
  uint8_t nonce[INIT_NONCE_SIZE];       // Client application nonce
  uint32_t cid;                         // Channel identifier  
  uint8_t versionInterface;             // Interface version
  uint8_t versionMajor;                 // Major version number
  uint8_t versionMinor;                 // Minor version number
  uint8_t versionBuild;                 // Build version number
  uint8_t capFlags;                     // Capabilities flags  
} __attribute__ ((packed)) U2FHID_INIT_RESP;

// U2FHID_SYNC command defines

typedef struct {
  uint8_t nonce;                        // Client application nonce
} __attribute__ ((packed)) U2FHID_SYNC_REQ;

typedef struct {
  uint8_t nonce;                        // Client application nonce
} __attribute__ ((packed)) U2FHID_SYNC_RESP;

// Low-level error codes. Return as negatives.

#define ERR_NONE                0x00    // No error
#define ERR_INVALID_CMD         0x01    // Invalid command
#define ERR_INVALID_PAR         0x02    // Invalid parameter
#define ERR_INVALID_LEN         0x03    // Invalid message length
#define ERR_INVALID_SEQ         0x04    // Invalid message sequencing
#define ERR_MSG_TIMEOUT         0x05    // Message has timed out
#define ERR_CHANNEL_BUSY        0x06    // Channel busy
#define ERR_LOCK_REQUIRED       0x0a    // Command requires channel lock
#define ERR_INVALID_CID         0x0b    // Invalid channel
#define ERR_SYNC_FAIL           0x0b    // SYNC command failed
#define ERR_OTHER               0x7f    // Other unspecified error


#define PRIO_U2F_HID      4

extern uint8_t __process4_stack_base__[], __process4_stack_size__[];
#define STACK_ADDR_U2F_HID ((uint32_t)__process4_stack_base__)
#define STACK_SIZE_U2F_HID ((uint32_t)__process4_stack_size__)

#define MAX_MSGLEN 1024
#define MAX_APDU_CMDLEN 160

struct u2f_hid {
  uint32_t next_cid;
  uint32_t cid;
  U2FHID_FRAME frame;
  struct usb_hid *hid;
  uint8_t cmd;
  uint8_t seq;
  uint8_t msg[MAX_MSGLEN];
  uint8_t apdu_cmd[MAX_APDU_CMDLEN];
  uint16_t msg_len;
  uint16_t msg_pos;
};

static struct u2f_hid u2f_hid;

static int
u2f_msg_init (struct u2f_hid *u2f, uint16_t len)
{
  u2f->msg_len = len;
  u2f->msg_pos = 0;

  return 0;
}

static int
u2f_msg_append (struct u2f_hid *u2f, uint8_t *buf, uint16_t len)
{
  if (u2f->msg_pos + len >= u2f->msg_len)
    len = u2f->msg_len - u2f->msg_pos;
  memcpy (u2f->msg + u2f->msg_pos, buf, len);
  u2f->msg_pos += len;
  return u2f->msg_len - u2f->msg_pos;
}

static void
u2f_send_error (struct u2f_hid *u2f, uint32_t cid, uint8_t error)
{
  memset (u2f->frame.init.data, 0, sizeof (u2f->frame.init.data));
  u2f->frame.cid = cid;
  u2f->frame.init.cmd = U2FHID_ERROR;
  u2f->frame.init.data[0] = error;
  SET_MSG_LEN (u2f->frame, 1);
  hid_send (u2f->hid, (uint8_t *) &u2f->frame, sizeof (u2f->frame));
}

static void
u2f_send_init (struct u2f_hid *u2f, uint32_t cid, uint32_t resp_cid,
               uint8_t *nonce)
{
  U2FHID_INIT_RESP *resp;

  u2f->frame.cid = cid;
  u2f->frame.init.cmd = U2FHID_INIT;
  SET_MSG_LEN (u2f->frame, sizeof (U2FHID_INIT_RESP));

  resp = (U2FHID_INIT_RESP *) &u2f->frame.init.data;
  memcpy (&resp->nonce, nonce, INIT_NONCE_SIZE);
  memset (u2f->frame.init.data + INIT_NONCE_SIZE, 0,
          sizeof (u2f->frame.init.data) - INIT_NONCE_SIZE);
  resp->cid = resp_cid;
  resp->versionInterface = U2FHID_IF_VERSION;
  resp->versionMajor = 1;
  resp->versionMinor = 1;
  resp->versionBuild = 1;
  resp->capFlags = 0;

  hid_send (u2f->hid, (uint8_t *) &u2f->frame, sizeof (u2f->frame));
}

static void
u2f_send_msg (struct u2f_hid *u2f, uint32_t cid, uint8_t cmd, uint8_t *msg,
              uint16_t len)
{
  uint16_t remain;
  uint16_t frame_len;
  uint8_t seq;

  remain = len;

  u2f->frame.cid = cid;
  u2f->frame.init.cmd = cmd;
  SET_MSG_LEN (u2f->frame, len);

  frame_len = remain;

  if (frame_len > sizeof (u2f->frame.init.data))
    frame_len = sizeof (u2f->frame.init.data);

  memset (u2f->frame.init.data, 0, sizeof (u2f->frame.init.data));
  memcpy (u2f->frame.init.data, msg, frame_len);

  hid_send (u2f->hid, (uint8_t *) &u2f->frame, sizeof (u2f->frame));

  remain -= frame_len;
  msg += frame_len;
  seq = 0;

  while (remain > 0)
    {
      u2f->frame.cont.seq = seq;

      frame_len = remain;

      if (frame_len > sizeof (u2f->frame.cont.data))
        frame_len = sizeof (u2f->frame.cont.data);

      memset (u2f->frame.cont.data, 0, sizeof (u2f->frame.cont.data));
      memcpy (u2f->frame.cont.data, msg, frame_len);

      hid_send (u2f->hid, (uint8_t *) &u2f->frame, sizeof (u2f->frame));

      remain -= frame_len;
      msg += frame_len;
      ++seq;
    }
}

static void
uf2_reset (struct u2f_hid *u2f)
{
  u2f->cmd = 0;
  u2f->cid = 0;
  u2f->seq = 0;
}

extern uint8_t blink_is_on;

static void *
u2f_hid_main (void *arg)
{
  struct u2f_hid *u2f = (struct u2f_hid *) arg;
  int remain = 0;
  int err;

  u2f_apdu_init ();

  while (1)
    {
      err = hid_recv (u2f->hid, (uint8_t *) &u2f->frame,
                      sizeof(u2f->frame), 500000);

      if (err == -1 && u2f->cmd)
        {
          u2f_send_error (u2f, u2f->cid, ERR_MSG_TIMEOUT);
          uf2_reset (u2f);
          continue;
        }

      if (err == -1)
        continue;

      blink_is_on = 1;

      if (u2f->frame.cid == 0 ||
          (u2f->frame.cid == CID_BROADCAST
           && u2f->frame.init.cmd != U2FHID_INIT))
        {
          u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_CID);
          continue;
        }

      if (u2f->frame.init.cmd == U2FHID_INIT)
        {
          if (MSG_LEN (u2f->frame) != sizeof (U2FHID_INIT_REQ))
            {
              u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_LEN);
              continue;
            }

          if (u2f->frame.cid == u2f->cid)
            {
              uf2_reset (u2f);
            }

          u2f_send_init (
            u2f,
            u2f->frame.cid,
            u2f->frame.cid == CID_BROADCAST ?
              u2f->next_cid : u2f->frame.cid,
            u2f->frame.init.data);

          u2f->next_cid += 2;

          continue;
        }

      if (FRAME_TYPE (u2f->frame) == TYPE_INIT &&
          u2f->cid != 0 && u2f->frame.cid != u2f->cid)
        {
          u2f_send_error (u2f, u2f->frame.cid, ERR_CHANNEL_BUSY);
          continue;
        }

      if (FRAME_TYPE (u2f->frame) == TYPE_INIT &&
          (u2f->cid == 0 || u2f->cid == u2f->frame.cid))
        {
          if (u2f->cmd)
            {
              u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_SEQ);
              uf2_reset (u2f);
              continue;
            }

          if (MSG_LEN (u2f->frame) > MAX_MSGLEN)
            {
              u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_LEN);
              uf2_reset (u2f);
              continue;
            }

          u2f->cid = u2f->frame.cid;
          u2f->cmd = u2f->frame.init.cmd;
          u2f_msg_init (u2f, MSG_LEN (u2f->frame));
          remain = u2f_msg_append (u2f, u2f->frame.init.data,
                                   sizeof (u2f->frame.init.data));
        }
      else if (FRAME_TYPE (u2f->frame) == TYPE_CONT &&
               u2f->cid == u2f->frame.cid)
        {
          if (u2f->frame.cont.seq != u2f->seq++)
            {
              u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_SEQ);
              uf2_reset (u2f);
              continue;
            }
          remain = u2f_msg_append (u2f, u2f->frame.cont.data,
                                   sizeof (u2f->frame.cont.data));
        }

      if (remain == 0 && u2f->cmd)
        {
          uint32_t resp_len;

          switch (u2f->cmd)
            {
              case U2FHID_PING:
                u2f_send_msg (u2f, u2f->cid, U2FHID_PING, u2f->msg,
                              u2f->msg_len);
                break;
              case U2FHID_MSG:
                if (u2f->msg_len > MAX_APDU_CMDLEN)
                  {
                    u2f_send_error (u2f, u2f->frame.cid, ERR_INVALID_LEN);
                    uf2_reset (u2f);
                    continue;
                  }
                memcpy (u2f->apdu_cmd, u2f->msg, u2f->msg_len);
                memset (u2f->msg, 0, MAX_MSGLEN);
                u2f_apdu_command_do (u2f->apdu_cmd, u2f->msg_len,
                                     u2f->msg, &resp_len);
                u2f_send_msg (u2f, u2f->cid, U2FHID_MSG, u2f->msg, resp_len);
                break;
              default:
                u2f_send_error (u2f, u2f->cid, ERR_INVALID_CMD);
            }
          uf2_reset (u2f);
        }
    }

  return NULL;
}

struct u2f_hid *
u2f_hid_open (struct usb_hid *hid)
{
  memset (&u2f_hid, 0, sizeof (struct u2f_hid));

  u2f_hid.next_cid = 0xdeadbeaf + 12;
  u2f_hid.hid = hid;

  chopstx_create (PRIO_U2F_HID, STACK_ADDR_U2F_HID, STACK_SIZE_U2F_HID,
                  u2f_hid_main, &u2f_hid);

  return &u2f_hid;
}
