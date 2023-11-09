/**
 * @file pem_decrypt.h
 * @brief PEM file decryption
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

#ifndef _PEM_DECRYPT_H
#define _PEM_DECRYPT_H

//Dependencies
#include "core/crypto.h"
#include "pkix/pem_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//PEM related functions
error_t pemDecryptPrivateKey(const char_t *input, size_t inputLen,
   const char_t *password, char_t *output, size_t *outputLen);

error_t pemDecryptMessage(const PemHeader *header, const char_t *password,
   const uint8_t *ciphertext, size_t ciphertextLen, uint8_t *plaintext,
   size_t *plaintextLen);

error_t pemFormatIv(const PemHeader *header, uint8_t *iv, size_t ivLen);

error_t pemKdf(const char_t *p, size_t pLen, const uint8_t *s, size_t sLen,
   uint8_t *dk, size_t dkLen);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
