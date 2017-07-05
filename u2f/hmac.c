#include <stdint.h>

#include "sha256.h"
#include "hmac.h"

static void
_hmac_sha256_init (const hmac_context *ctx)
{
  hmac_sha256_context *context = (hmac_sha256_context *)ctx;
  sha256_start(&context->sha_ctx);
}

static void
_hmac_sha256_update (const hmac_context *ctx, const uint8_t *message,
                    unsigned message_size)
{
  hmac_sha256_context *context = (hmac_sha256_context *)ctx;
  sha256_update(&context->sha_ctx, message, message_size);
}

static void
_hmac_sha256_finish (const hmac_context *ctx, uint8_t *hash_result)
{
  hmac_sha256_context *context = (hmac_sha256_context *)ctx;
  sha256_finish(&context->sha_ctx, hash_result);
}

/* Compute an HMAC using K as a key (as in RFC 6979). Note that K is always
   the same size as the hash result size. */
static void
hmac_init(const hmac_context *ctx, const uint8_t *K)
{
  uint8_t *pad = ctx->tmp + 2 * ctx->result_size;
  unsigned i;
  for (i = 0; i < ctx->result_size; ++i)
    pad[i] = K[i] ^ 0x36;
  for (; i < ctx->block_size; ++i)
    pad[i] = 0x36;

  ctx->init_hash (ctx);
  ctx->update_hash (ctx, pad, ctx->block_size);
}

static void
hmac_update(const hmac_context *ctx,
            const uint8_t *message,
            unsigned message_size)
{
  ctx->update_hash (ctx, message, message_size);
}

static void
hmac_finish(const hmac_context *ctx,
            const uint8_t *K,
            uint8_t *result)
{
  uint8_t *pad = ctx->tmp + 2 * ctx->result_size;
  unsigned i;
  for (i = 0; i < ctx->result_size; ++i)
    pad[i] = K[i] ^ 0x5c;
  for (; i < ctx->block_size; ++i)
    pad[i] = 0x5c;

  ctx->finish_hash (ctx, result);

  ctx->init_hash (ctx);
  ctx->update_hash (ctx, pad, ctx->block_size);
  ctx->update_hash (ctx, result, ctx->result_size);
  ctx->finish_hash (ctx, result);
}

void
hmac_sha256_init (hmac_sha256_context *ctx, const uint8_t *K)
{
  ctx->hmac_ctx.init_hash = _hmac_sha256_init;
  ctx->hmac_ctx.update_hash = _hmac_sha256_update;
  ctx->hmac_ctx.finish_hash = _hmac_sha256_finish;
  ctx->hmac_ctx.block_size = 64;
  ctx->hmac_ctx.result_size = 32;
  ctx->hmac_ctx.tmp = ctx->tmp;
  hmac_init (&ctx->hmac_ctx, K);
}

void
hmac_sha256_update (const hmac_sha256_context *ctx, const uint8_t *message,
                    unsigned message_size)
{
  hmac_update (&ctx->hmac_ctx, message, message_size);
}

void
hmac_sha256_finish (const hmac_sha256_context *ctx, const uint8_t *K,
                    uint8_t *hash_result)
{
  hmac_finish (&ctx->hmac_ctx, K, hash_result);
}
