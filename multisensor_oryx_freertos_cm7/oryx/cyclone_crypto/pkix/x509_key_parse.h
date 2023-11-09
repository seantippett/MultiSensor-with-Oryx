/**
 * @file x509_key_parse.h
 * @brief Parsing of ASN.1 encoded keys
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

#ifndef _X509_KEY_PARSE_H
#define _X509_KEY_PARSE_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Key parsing functions
error_t x509ParseSubjectPublicKeyInfo(const uint8_t *data, size_t length,
   size_t *totalLength, X509SubjectPublicKeyInfo *subjectPublicKeyInfo);

error_t x509ParseAlgoId(const uint8_t *data, size_t length,
   size_t *totalLength, X509SubjectPublicKeyInfo *subjectPublicKeyInfo);

error_t x509ParseRsaPublicKey(const uint8_t *data, size_t length,
   X509RsaPublicKey *rsaPublicKey);

error_t x509ParseDsaPublicKey(const uint8_t *data, size_t length,
   X509DsaPublicKey *dsaPublicKey);

error_t x509ParseDsaParameters(const uint8_t *data, size_t length,
   X509DsaParameters *dsaParams);

error_t x509ParseEcPublicKey(const uint8_t *data, size_t length,
   X509EcPublicKey *ecPublicKey);

error_t x509ParseEcParameters(const uint8_t *data, size_t length,
   X509EcParameters *ecParams);

error_t x509ImportRsaPublicKey(const X509SubjectPublicKeyInfo *publicKeyInfo,
   RsaPublicKey *publicKey);

error_t x509ImportDsaPublicKey(const X509SubjectPublicKeyInfo *publicKeyInfo,
   DsaPublicKey *publicKey);

error_t x509ImportEcPublicKey(const X509SubjectPublicKeyInfo *publicKeyInfo,
   EcPublicKey *publicKey);

error_t x509ImportEcParameters(const X509EcParameters *ecParams,
   EcDomainParameters *params);

error_t x509ImportEddsaPublicKey(const X509SubjectPublicKeyInfo *publicKeyInfo,
   EddsaPublicKey *publicKey);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
