/**
 * @file tls_record_decryption.h
 * @brief TLS record decryption
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

#ifndef _TLS_RECORD_DECRYPTION_H
#define _TLS_RECORD_DECRYPTION_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsDecryptRecord(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

error_t tlsDecryptAeadRecord(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

error_t tlsDecryptCbcRecord(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

error_t tlsDecryptStreamRecord(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

error_t tlsVerifyMessageAuthCode(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

uint32_t tlsVerifyPadding(const uint8_t *data, size_t dataLen,
   size_t *paddingLen);

uint32_t tlsVerifyMac(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, const void *record,
   const uint8_t *data, size_t dataLen, size_t maxDataLen, const uint8_t *mac);

uint32_t tlsExtractMac(TlsEncryptionEngine *decryptionEngine,
   const uint8_t *data, size_t dataLen, size_t maxDataLen, uint8_t *mac);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
