/**
 * @file tls_key_material.h
 * @brief Key material generation
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

#ifndef _TLS_KEY_MATERIAL_H
#define _TLS_KEY_MATERIAL_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsGenerateSessionKeys(TlsContext *context);
error_t tlsGenerateMasterSecret(TlsContext *context);
error_t tlsGenerateExtendedMasterSecret(TlsContext *context);
error_t tlsGeneratePskPremasterSecret(TlsContext *context);
error_t tlsGenerateKeyBlock(TlsContext *context, size_t keyBlockLen);

error_t tlsExportKeyingMaterial(TlsContext *context, const char_t *label,
   bool_t useContextValue, const uint8_t *contextValue,
   size_t contextValueLen, uint8_t *output, size_t outputLen);

error_t tlsPrf(const uint8_t *secret, size_t secretLen, const char_t *label,
   const uint8_t *seed, size_t seedLen, uint8_t *output, size_t outputLen);

error_t tls12Prf(const HashAlgo *hash, const uint8_t *secret, size_t secretLen,
   const char_t *label, const uint8_t *seed, size_t seedLen, uint8_t *output,
   size_t outputLen);

void tlsDumpSecret(TlsContext *context, const char_t *label,
   const uint8_t *secret, size_t secretLen);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
