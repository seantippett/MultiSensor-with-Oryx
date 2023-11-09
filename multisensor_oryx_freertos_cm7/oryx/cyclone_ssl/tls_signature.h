/**
 * @file tls_signature.h
 * @brief RSA/DSA/ECDSA/EdDSA signature generation and verification (TLS 1.3)
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

#ifndef _TLS_SIGNATURE_H
#define _TLS_SIGNATURE_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsSelectSignatureScheme(TlsContext *context, const TlsCertDesc *cert,
   const TlsSignHashAlgos *supportedSignAlgos);

error_t tlsGenerateSignature(TlsContext *context, uint8_t *p,
   size_t *length);

error_t tlsVerifySignature(TlsContext *context, const uint8_t *p,
   size_t length);

error_t tls12GenerateSignature(TlsContext *context, uint8_t *p,
   size_t *length);

error_t tls12VerifySignature(TlsContext *context, const uint8_t *p,
   size_t length);

error_t tlsGenerateRsaSignature(const RsaPrivateKey *key,
   const uint8_t *digest, uint8_t *signature, size_t *signatureLen);

error_t tlsVerifyRsaSignature(const RsaPublicKey *key,
   const uint8_t *digest, const uint8_t *signature, size_t signatureLen);

error_t tlsVerifyRsaEm(const uint8_t *digest, const uint8_t *em, size_t emLen);

error_t tlsGenerateDsaSignature(TlsContext *context, const uint8_t *digest,
   size_t digestLen, uint8_t *signature, size_t *signatureLen);

error_t tlsVerifyDsaSignature(TlsContext *context, const uint8_t *digest,
   size_t digestLen, const uint8_t *signature, size_t signatureLen);

error_t tlsGenerateEcdsaSignature(TlsContext *context, const uint8_t *digest,
   size_t digestLen, uint8_t *signature, size_t *signatureLen);

error_t tlsVerifyEcdsaSignature(TlsContext *context, const uint8_t *digest,
   size_t digestLen, const uint8_t *signature, size_t signatureLen);

error_t tlsGenerateEddsaSignature(TlsContext *context,
   const EddsaMessageChunk *messageChunks, uint8_t *signature,
   size_t *signatureLen);

error_t tlsVerifyEddsaSignature(TlsContext *context,
   const EddsaMessageChunk *messageChunks, const uint8_t *signature,
   size_t signatureLen);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
