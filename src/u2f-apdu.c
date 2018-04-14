/*
 * u2f-apdu.c - U2F apdu commands
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
#include "sha256.h"
#include "ecc.h"
#include "hmac.h"
#include "random.h"
#include "board.h"
#include "sys.h"
#include "pbt.h"

#define CLA(apdu)  ((apdu)[0])
#define INS(apdu)  ((apdu)[1])
#define P1(apdu)   ((apdu)[2])
#define P2(apdu)   ((apdu)[3])
#define LC(apdu)   (((apdu)[4] << 16) | ((apdu)[5] << 8) | (apdu)[6])
#define DATA(apdu) ((apdu) + 7)


// General constants

#define HASH_BLOCK_SIZE                 64
#define HASH_RES_SIZE                   32

#define U2F_EC_KEY_SIZE         32      // EC key size in bytes
#define U2F_EC_POINT_SIZE       ((U2F_EC_KEY_SIZE * 2) + 1) // Size of EC point
#define U2F_NONCE_SIZE          32      // size of nonce
#define U2F_KH_SIZE             (HASH_RES_SIZE + U2F_NONCE_SIZE) // size of key handle
#define U2F_PRIV_K_SIZE         32      // size of private key
#define U2F_PUB_K_SIZE          64      // size of public key
#define U2F_MAX_EC_SIG_SIZE     72      // Max size of DER coded EC signature
#define U2F_CTR_SIZE            4       // Size of counter field
#define U2F_APPID_SIZE          32      // Size of application id
#define U2F_CHAL_SIZE           32      // Size of challenge

#define ENC_SIZE(x)             ((x + 7) & 0xfff8)

// EC (uncompressed) point

#define U2F_POINT_UNCOMPRESSED  0x04    // Uncompressed point format

typedef struct {
    uint8_t pointFormat;                // Point type
    uint8_t x[U2F_EC_KEY_SIZE];         // X-value
    uint8_t y[U2F_EC_KEY_SIZE];         // Y-value
} U2F_EC_POINT;

// U2F native commands

#define U2F_REGISTER            0x01    // Registration command
#define U2F_AUTHENTICATE        0x02    // Authenticate/sign command
#define U2F_VERSION             0x03    // Read version string command

#define U2F_VENDOR_FIRST        0x40    // First vendor defined command
#define U2F_VENDOR_LAST         0xbf    // Last vendor defined command

// U2F_CMD_REGISTER command defines

#define U2F_REGISTER_ID         0x05    // Version 2 registration identifier
#define U2F_REGISTER_HASH_ID    0x00    // Version 2 hash identintifier

#include "cert/certificates.c"

typedef struct {
    uint8_t chal[U2F_CHAL_SIZE];        // Challenge
    uint8_t appId[U2F_APPID_SIZE];      // Application id
} __attribute__ ((packed)) U2F_REGISTER_REQ;

typedef struct {
    uint8_t registerId;                 // Registration identifier (U2F_REGISTER_ID_V2)
    U2F_EC_POINT pubKey;                // Generated public key
    uint8_t keyHandleLen;               // Length of key handle
    uint8_t keyHandle[U2F_KH_SIZE];     // Key handle
    uint8_t attCert[ATTESTATION_DER_LEN]; // Attestation certificate
    uint8_t sig[U2F_MAX_EC_SIG_SIZE];   // Registration signature
    uint8_t pad[2];                     // For SW1 and SW2
} __attribute__ ((packed)) U2F_REGISTER_RESP;

// U2F_CMD_AUTHENTICATE command defines

// Authentication control byte

#define U2F_AUTH_ENFORCE        0x03    // Enforce user presence and sign
#define U2F_AUTH_CHECK_ONLY     0x07    // Check only
#define U2F_AUTH_FLAG_TUP       0x01    // Test of user presence set

typedef struct {
    uint8_t chal[U2F_CHAL_SIZE];        // Challenge
    uint8_t appId[U2F_APPID_SIZE];      // Application id
    uint8_t keyHandleLen;               // Length of key handle
    uint8_t keyHandle[U2F_KH_SIZE];     // Key handle
} __attribute__ ((packed)) U2F_AUTHENTICATE_REQ;

typedef struct {
    uint8_t flags;                      // U2F_AUTH_FLAG_ values
    uint8_t ctr[U2F_CTR_SIZE];          // Counter field (big-endian)
    uint8_t sig[U2F_MAX_EC_SIG_SIZE];   // Signature
    uint8_t pad[2];                     // For SW1 and SW2
} __attribute__ ((packed)) U2F_AUTHENTICATE_RESP;

// Command status responses

#define U2F_SW_NO_ERROR                 0x9000 // SW_NO_ERROR
#define U2F_SW_WRONG_DATA               0x6A80 // SW_WRONG_DATA
#define U2F_SW_CONDITIONS_NOT_SATISFIED 0x6985 // SW_CONDITIONS_NOT_SATISFIED
#define U2F_SW_COMMAND_NOT_ALLOWED      0x6986 // SW_COMMAND_NOT_ALLOWED
#define U2F_SW_INS_NOT_SUPPORTED        0x6D00 // SW_INS_NOT_SUPPORTED
#define U2F_SW_WRONG_LENGTH             0x6700 // SW_INS_WRONG_LENGTH
#define U2F_SW_BAD_CLA                  0x6E00 // SW_BAD_CLA

/* simple RNG interface */

static int
rng (uint8_t *buf, size_t size)
{
  uint8_t rng_index = 0;
  int res;

  res = random_gen (&rng_index, buf, size);
  random_bytes_free (NULL);   /* parameter isn't used. invokes neug_flush. */

  return res;
}

/* device key */

struct device_key
{
  /* ECC private key unique for each device */
  uint8_t key[U2F_PRIV_K_SIZE];
  /* SHA256 of private key to verify its integrity */
  uint8_t key_hash[HASH_RES_SIZE];
  /* reserved for future use */
  uint8_t resrved[1024 - U2F_PRIV_K_SIZE - HASH_RES_SIZE];
};

struct device_key __attribute__ ((section(".device.key"))) device_key = { 0 };

uint32_t __attribute__ ((section(".auth.ctr"))) auth_ctr[256] = { 0 };
static uint32_t *ctr_addr = &(auth_ctr[0]);

static void
device_key_gen (void)
{
  uint8_t key[U2F_PRIV_K_SIZE];
  uint8_t key_hash[HASH_RES_SIZE];
  sha256_context ctx;

  sha256_start (&ctx);
  sha256_update (&ctx, device_key.key, U2F_PRIV_K_SIZE);
  sha256_finish (&ctx, key_hash);

  if (memcmp (key_hash, device_key.key_hash, HASH_RES_SIZE) == 0)
    return;

  /* new device key needs to be generated */
  rng (key, U2F_PRIV_K_SIZE);

  sha256_start (&ctx);
  sha256_update (&ctx, key, U2F_PRIV_K_SIZE);
  sha256_finish (&ctx, key_hash);

  /* write device key to flash */
  flash_erase_page ((uintptr_t) &device_key);
  flash_write ((uintptr_t) device_key.key, key, U2F_PRIV_K_SIZE);
  flash_write ((uintptr_t) device_key.key_hash, key_hash, HASH_RES_SIZE);

  /* erase auth counter */
  flash_erase_page ((uintptr_t) ctr_addr);
}


static void
new_private_key (uint8_t *app_id, uint8_t *nonce, uint8_t *private_key)
{
  hmac_sha256_context ctx;

  hmac_sha256_init (&ctx, device_key.key);
  hmac_sha256_update (&ctx, app_id, U2F_APPID_SIZE);
  hmac_sha256_update (&ctx, nonce, U2F_NONCE_SIZE);
  hmac_sha256_finish (&ctx, device_key.key, private_key);
}

static void
make_key_handle (uint8_t *private_key, uint8_t *app_id,
                 uint8_t *nonce, uint8_t *key_handle)
{
  hmac_sha256_context ctx;

  hmac_sha256_init (&ctx, device_key.key);
  hmac_sha256_update (&ctx, private_key, U2F_PRIV_K_SIZE);
  hmac_sha256_update (&ctx, app_id, U2F_APPID_SIZE);
  hmac_sha256_finish (&ctx, device_key.key, key_handle);

  memcpy (key_handle + HASH_RES_SIZE, nonce, U2F_NONCE_SIZE);
}

static int
recover_private_key (uint8_t *app_id, uint8_t *key_handle,
                     uint8_t key_handle_len, uint8_t *private_key)
{
  hmac_sha256_context ctx;
  uint8_t control_mac[HASH_RES_SIZE];

  if (key_handle_len != U2F_KH_SIZE)
    return -1;

  hmac_sha256_init (&ctx, device_key.key);
  hmac_sha256_update (&ctx, app_id, U2F_APPID_SIZE);
  hmac_sha256_update (&ctx, key_handle + HASH_RES_SIZE, U2F_NONCE_SIZE);
  hmac_sha256_finish (&ctx, device_key.key, private_key);

  hmac_sha256_init (&ctx, device_key.key);
  hmac_sha256_update (&ctx, private_key, U2F_PRIV_K_SIZE);
  hmac_sha256_update (&ctx, app_id, U2F_APPID_SIZE);
  hmac_sha256_finish (&ctx, device_key.key, control_mac);

  return memcmp(control_mac, key_handle, HASH_RES_SIZE);
}

static uint8_t
der_encode_uint (uint8_t *der, uint8_t *x, uint8_t x_len)
{
  uint8_t len = 0;

  /* integer */
  der[len++] = 0x02;

  /* reserve field for length, works only for short form */
  der[len++] = 0;

  /* omit leading zeros */
  while (x[0] == 0 && x_len > 0)
    {
      ++x;
      --x_len;
    }

  /* positive integer: 8th bit must be zero */
  if (x[0] > 0x7f)
    der[len++] = 0;

  /* copy actual data */
  memcpy (der + len, x, x_len);
  len += x_len;

  /* set length field */
  der[1] = len - 2;

  return len;
}

static uint8_t
der_encode_sig (uint8_t *der, uint8_t *sig)
{
  uint8_t len;

  len = 0;

  der[len++] = 0x30;

  /* reserve length field */
  der[len++] = 0;

  len += der_encode_uint (der + len, sig, 32);
  len += der_encode_uint (der + len, sig + 32, 32);

  der[1] = len - 2;

  return len;
}

static void
register_req_hash (U2F_REGISTER_REQ *req, U2F_REGISTER_RESP *resp,
                   uint8_t *hash)
{
  sha256_context ctx;
  uint8_t resrved = 0;

  sha256_start (&ctx);
  sha256_update (&ctx, &resrved, 1);
  sha256_update (&ctx, req->appId, U2F_APPID_SIZE);
  sha256_update (&ctx, req->chal, U2F_CHAL_SIZE);
  sha256_update (&ctx, resp->keyHandle, resp->keyHandleLen);
  sha256_update (&ctx, (uint8_t *) &resp->pubKey, sizeof (resp->pubKey));
  sha256_finish (&ctx, hash);
}

static int
u2f_register (U2F_REGISTER_REQ *req, U2F_REGISTER_RESP *resp)
{
  uint8_t private[U2F_PRIV_K_SIZE];
  uint8_t nonce[U2F_NONCE_SIZE];
  uint8_t hash[HASH_RES_SIZE];
  uint8_t sig[64];
  uint8_t sig_len;

  if (rng (nonce, U2F_NONCE_SIZE))
    return -1;

  new_private_key (req->appId, nonce, private);
  make_key_handle (private, req->appId, nonce, resp->keyHandle);

  if (ecc_compute_public_p256r1 (private, resp->pubKey.x))
    return -1;

  resp->registerId = U2F_REGISTER_ID;
  resp->pubKey.pointFormat = U2F_POINT_UNCOMPRESSED;
  resp->keyHandleLen = U2F_KH_SIZE;
  memcpy (resp->attCert, attestation_der, ATTESTATION_DER_LEN);

  register_req_hash (req, resp, hash);

  if (ecdsa_sign_p256r1 (hash, sig, attestation_key))
    return -1;

  sig_len = der_encode_sig (resp->sig, sig);

  return 1                        // Registration identifier (U2F_REGISTER_ID_V2)
         + sizeof (U2F_EC_POINT)  // Generated public key
         + 1                      // Length of key handle
         + U2F_KH_SIZE            // Key handle
         + ATTESTATION_DER_LEN    // Attestation certificate
         + sig_len;               // Registration signature
}

static uint32_t
u2f_read_ctr (void)
{
  const int page_size = 1024 / sizeof (*ctr_addr);

  while (*ctr_addr != 0xffffffff)
    {
      if (ctr_addr - &(auth_ctr[0]) == page_size)
        break;
      ctr_addr++;
    }

  if (ctr_addr == &(auth_ctr[0]))
    return 0;

  return ctr_addr[-1];
}

static void
u2f_write_ctr (uint32_t val)
{
  const int page_size = 1024 / sizeof (*ctr_addr);

  while (*ctr_addr != 0xffffffff)
    {
      if (ctr_addr - &(auth_ctr[0]) == page_size)
        {
          flash_erase_page ((uintptr_t) &(auth_ctr[0]));
          ctr_addr = &(auth_ctr[0]);
          break;
        }
      ctr_addr++;
    }

  flash_write ((uintptr_t) ctr_addr, (uint8_t *) &val, sizeof (val));
}

static void
u2f_inc_ctr (void)
{
  uint32_t ctr;

  ctr = u2f_read_ctr ();
  ctr++;
  u2f_write_ctr (ctr);
}

static void
auth_req_hash (U2F_AUTHENTICATE_REQ *req, U2F_AUTHENTICATE_RESP *resp,
               uint8_t *hash)
{
  sha256_context ctx;

  sha256_start (&ctx);
  sha256_update (&ctx, req->appId, U2F_APPID_SIZE);
  sha256_update (&ctx, &resp->flags, 1);
  sha256_update (&ctx, resp->ctr, 4);
  sha256_update (&ctx, req->chal, U2F_CHAL_SIZE);
  sha256_finish (&ctx, hash);
}

static int
u2f_authenticate (U2F_AUTHENTICATE_REQ *req, U2F_AUTHENTICATE_RESP *resp)
{
  uint8_t private[U2F_PRIV_K_SIZE];
  uint8_t hash[HASH_RES_SIZE];
  uint8_t sig[64];
  uint8_t sig_len;
  uint32_t ctr;

  resp->flags = U2F_AUTH_FLAG_TUP;

  ctr = u2f_read_ctr ();

  resp->ctr[0] = ctr >> 24 & 0xff;
  resp->ctr[1] = ctr >> 16 & 0xff;
  resp->ctr[2] = ctr >>  8 & 0xff;
  resp->ctr[3] = ctr       & 0xff;

  auth_req_hash (req, resp, hash);

  if (recover_private_key (req->appId, req->keyHandle,
                           req->keyHandleLen, private))
    return -1;

  if (ecdsa_sign_p256r1 (hash, sig, private))
    return -1;

  sig_len = der_encode_sig (resp->sig, sig);

  return 1                        // U2F_AUTH_FLAG_ values
         + U2F_CTR_SIZE           // Counter field (big-endian)
         + sig_len;               // Signature
}

static int
u2f_version (uint8_t *resp)
{
  memcpy (resp, "U2F_V2", 6);
  return 6;
}

static void
append_sw (uint8_t *msg, uint32_t *len, uint16_t sw)
{
  msg[(*len)++] = (sw >> 8) & 0xff;
  msg[(*len)++] = sw & 0xff;
}

static void
u2f_apdu_error (uint8_t *msg, uint32_t *len, uint16_t sw)
{
  *len = 0;
  append_sw (msg, len, sw);
}

void
u2f_apdu_init (void)
{
  /* generate and store device private key on first run */
  device_key_gen ();
}

int
u2f_apdu_command_do (uint8_t *apdu, uint8_t len,
                     uint8_t *resp, uint32_t *resp_len)
{
  uint32_t Lc;
  int ret;

  Lc = LC (apdu);

  if (CLA(apdu) != 0)
    {
      u2f_apdu_error (resp, resp_len, U2F_SW_BAD_CLA);
      return 0;
    }

  if (Lc + 7 > len)
    {
      u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_LENGTH);
      return 0;
    }

  switch (INS (apdu))
    {
    case U2F_REGISTER:
      if (Lc != sizeof (U2F_REGISTER_REQ))
        {
          u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_LENGTH);
          break;
        }
      if (!user_presence_get ())
        {
          u2f_apdu_error (resp, resp_len, U2F_SW_CONDITIONS_NOT_SATISFIED);
          return 0;
        }
      ret = u2f_register ((U2F_REGISTER_REQ *) DATA (apdu),
                          (U2F_REGISTER_RESP *) resp);
      if (ret > 0)
        {
          user_presence_reset ();
          *resp_len = ret;
          append_sw (resp, resp_len, U2F_SW_NO_ERROR);
        }
      else
        {
          u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_DATA);
        }
      break;
    case U2F_AUTHENTICATE:
      if (Lc != sizeof (U2F_AUTHENTICATE_REQ))
        {
          u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_LENGTH);
          break;
        }
      ret = u2f_authenticate ((U2F_AUTHENTICATE_REQ *) DATA (apdu),
                              (U2F_AUTHENTICATE_RESP *)resp);
      if (ret > 0)
        {
          if (P1 (apdu) != U2F_AUTH_CHECK_ONLY)
            {
            if (!user_presence_get ())
              {
                u2f_apdu_error (resp, resp_len,
                                U2F_SW_CONDITIONS_NOT_SATISFIED);
                return 0;
              }
              u2f_inc_ctr ();
              user_presence_reset ();
              *resp_len = ret;
              append_sw (resp, resp_len, U2F_SW_NO_ERROR);
            }
          else
            u2f_apdu_error (resp, resp_len, U2F_SW_CONDITIONS_NOT_SATISFIED);
        }
      else
        u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_DATA);
      break;
    case U2F_VERSION:
      if (Lc > 0)
        {
          u2f_apdu_error (resp, resp_len, U2F_SW_WRONG_LENGTH);
          break;
        }
      ret = u2f_version (resp);
      if (ret > 0)
        {
          *resp_len = ret;
          append_sw (resp, resp_len, U2F_SW_NO_ERROR);
        }
      break;
    default:
      u2f_apdu_error (resp, resp_len, U2F_SW_INS_NOT_SUPPORTED);
    }

  return 0;
}

