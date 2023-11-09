/**
 * @file pkcs5_decrypt.h
 * @brief PKCS #5 decryption routines
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

#ifndef _PKCS5_DECRYPT_H
#define _PKCS5_DECRYPT_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"
#include "pkix/pkcs5_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//PKCS #5 related functions
error_t pkcs5Decrypt(const X509AlgoId *encryptionAlgoId,
   const char_t *password, const uint8_t *ciphertext, size_t ciphertextLen,
   uint8_t *plaintext, size_t *plaintextLen);

error_t pkcs5DecryptPbes1(const X509AlgoId *encryptionAlgoId,
   const char_t *password, const uint8_t *ciphertext, size_t ciphertextLen,
   uint8_t *plaintext, size_t *plaintextLen);

error_t pkcs5DecryptPbes2(const X509AlgoId *encryptionAlgoId,
   const char_t *password, const uint8_t *ciphertext, size_t ciphertextLen,
   uint8_t *plaintext, size_t *plaintextLen);

error_t pkcs5ParsePbes1Params(const uint8_t *data, size_t length,
   Pkcs5Pbes1Params *pbes1Params);

error_t pkcs5ParsePbes2Params(const uint8_t *data, size_t length,
   Pkcs5Pbes2Params *pbes2Params);

error_t pkcs5ParseKeyDerivationFunc(const uint8_t *data, size_t length,
   size_t *totalLength, Pkcs5KeyDerivationFunc *keyDerivationFunc);

error_t pkcs5ParsePbkdf2Params(const uint8_t *data, size_t length,
   Pkcs5KeyDerivationFunc *keyDerivationFunc);

error_t pkcs5ParseEncryptionScheme(const uint8_t *data, size_t length,
   size_t *totalLength, Pkcs5EncryptionScheme *encryptionScheme);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
