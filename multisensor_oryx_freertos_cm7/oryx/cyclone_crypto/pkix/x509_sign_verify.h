/**
 * @file x509_sign_verify.h
 * @brief RSA/DSA/ECDSA/EdDSA signature verification
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

#ifndef _X509_SIGN_VERIFY_H
#define _X509_SIGN_VERIFY_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Signature verification callback function
 **/

typedef error_t (*X509SignVerifyCallback)(const X509OctetString *tbsData,
   const X509SignAlgoId *signAlgoId,
   const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);


//X.509 related functions
error_t x509RegisterSignVerifyCallback(X509SignVerifyCallback callback);

error_t x509VerifySignature(const X509OctetString *tbsData,
   const X509SignAlgoId *signAlgoId,
   const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyRsaSignature(const X509OctetString *tbsData,
   const HashAlgo *hashAlgo, const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyRsaPssSignature(const X509OctetString *tbsData,
   const HashAlgo *hashAlgo, size_t saltLen,
   const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyDsaSignature(const X509OctetString *tbsData,
   const HashAlgo *hashAlgo, const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyEcdsaSignature(const X509OctetString *tbsData,
   const HashAlgo *hashAlgo, const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyEd25519Signature(const X509OctetString *tbsData,
   const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

error_t x509VerifyEd448Signature(const X509OctetString *tbsData,
   const X509SubjectPublicKeyInfo *publicKeyInfo,
   const X509OctetString *signature);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
