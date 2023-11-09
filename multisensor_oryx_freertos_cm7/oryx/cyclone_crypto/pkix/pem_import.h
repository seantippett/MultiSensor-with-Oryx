/**
 * @file pem_import.h
 * @brief PEM file import functions
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

#ifndef _PEM_IMPORT_H
#define _PEM_IMPORT_H

//Dependencies
#include "core/crypto.h"
#include "pkix/pem_common.h"
#include "pkix/x509_common.h"
#include "pkc/dh.h"
#include "pkc/rsa.h"
#include "pkc/dsa.h"
#include "ecc/ec.h"
#include "ecc/eddsa.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//PEM related functions
error_t pemImportCertificate(const char_t *input, size_t inputLen,
   uint8_t *output, size_t *outputLen, size_t *consumed);

error_t pemImportCrl(const char_t *input, size_t inputLen,
   uint8_t *output, size_t *outputLen, size_t *consumed);

error_t pemImportCsr(const char_t *input, size_t inputLen,
   uint8_t *output, size_t *outputLen);

error_t pemImportDhParameters(const char_t *input, size_t length,
   DhParameters *params);

error_t pemImportRsaPublicKey(const char_t *input, size_t length,
   RsaPublicKey *publicKey);

error_t pemImportRsaPrivateKey(const char_t *input, size_t length,
   const char_t *password, RsaPrivateKey *privateKey);

error_t pemImportDsaPublicKey(const char_t *input, size_t length,
   DsaPublicKey *publicKey);

error_t pemImportDsaPrivateKey(const char_t *input, size_t length,
   const char_t *password, DsaPrivateKey *privateKey);

error_t pemImportEcParameters(const char_t *input, size_t length,
   EcDomainParameters *params);

error_t pemImportEcPublicKey(const char_t *input, size_t length,
   EcPublicKey *publicKey);

error_t pemImportEcPrivateKey(const char_t *input, size_t length,
   const char_t *password, EcPrivateKey *privateKey);

error_t pemImportEddsaPublicKey(const char_t *input, size_t length,
   EddsaPublicKey *publicKey);

error_t pemImportEddsaPrivateKey(const char_t *input, size_t length,
   const char_t *password, EddsaPrivateKey *privateKey);

error_t pemGetPublicKeyType(const char_t *input, size_t length,
   X509KeyType *keyType);

error_t pemGetPrivateKeyType(const char_t *input, size_t length,
   X509KeyType *keyType);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
