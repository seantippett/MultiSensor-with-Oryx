/**
 * @file hmac.h
 * @brief HMAC (Keyed-Hashing for Message Authentication)
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSSL Eval.
 *
 * This software is provided in source form for a short-term evaluation only. The
 * evaluation license expires 90 days after the date you first download the software.
 *
 * If you plan to use this software in a commercial product, you are required to
 * purchase a commercial license from Oryx Embedded SARL.
 *
 * After the 90-day evaluation period, you agree to either purchase a commercial
 * license or delete all copies of this software. If you wish to extend the
 * evaluation period, you must contact sales@oryx-embedded.com.
 *
 * This evaluation software is provided "as is" without warranty of any kind.
 * Technical support is available as an option during the evaluation period.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _HMAC_H
#define _HMAC_H

//Dependencies
#include "core/crypto.h"
#include "hash/hash_algorithms.h"

//Application specific context
#ifndef HMAC_PRIVATE_CONTEXT
   #define HMAC_PRIVATE_CONTEXT
#endif

//Inner padding (ipad)
#define HMAC_IPAD 0x36
//Outer padding (opad)
#define HMAC_OPAD 0x5C

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief HMAC algorithm context
 **/

typedef struct
{
   const HashAlgo *hash;
   HashContext hashContext;
   uint8_t key[MAX_HASH_BLOCK_SIZE];
   uint8_t digest[MAX_HASH_DIGEST_SIZE];
   HMAC_PRIVATE_CONTEXT
} HmacContext;


//HMAC related constants
extern const uint8_t HMAC_WITH_MD5_OID[8];
extern const uint8_t HMAC_WITH_TIGER_OID[8];
extern const uint8_t HMAC_WITH_RIPEMD160_OID[8];
extern const uint8_t HMAC_WITH_SHA1_OID[8];
extern const uint8_t HMAC_WITH_SHA224_OID[8];
extern const uint8_t HMAC_WITH_SHA256_OID[8];
extern const uint8_t HMAC_WITH_SHA384_OID[8];
extern const uint8_t HMAC_WITH_SHA512_OID[8];
extern const uint8_t HMAC_WITH_SHA512_224_OID[8];
extern const uint8_t HMAC_WITH_SHA512_256_OID[8];
extern const uint8_t HMAC_WITH_SHA3_224_OID[9];
extern const uint8_t HMAC_WITH_SHA3_256_OID[9];
extern const uint8_t HMAC_WITH_SHA3_384_OID[9];
extern const uint8_t HMAC_WITH_SHA3_512_OID[9];
extern const uint8_t HMAC_WITH_SM3_OID[10];

//HMAC related functions
error_t hmacCompute(const HashAlgo *hash, const void *key, size_t keyLen,
   const void *data, size_t dataLen, uint8_t *digest);

error_t hmacInit(HmacContext *context, const HashAlgo *hash,
   const void *key, size_t keyLen);

void hmacUpdate(HmacContext *context, const void *data, size_t length);
void hmacFinal(HmacContext *context, uint8_t *digest);
void hmacFinalRaw(HmacContext *context, uint8_t *digest);
void hmacDeinit(HmacContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
