/**
 * @file tls_record_encryption.h
 * @brief TLS record encryption
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

#ifndef _TLS_RECORD_ENCRYPTION_H
#define _TLS_RECORD_ENCRYPTION_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsEncryptRecord(TlsContext *context,
   TlsEncryptionEngine *encryptionEngine, void *record);

error_t tlsEncryptAeadRecord(TlsContext *context,
   TlsEncryptionEngine *encryptionEngine, void *record);

error_t tlsEncryptCbcRecord(TlsContext *context,
   TlsEncryptionEngine *encryptionEngine, void *record);

error_t tlsEncryptStreamRecord(TlsContext *context,
   TlsEncryptionEngine *encryptionEngine, void *record);

error_t tlsAppendMessageAuthCode(TlsContext *context,
   TlsEncryptionEngine *decryptionEngine, void *record);

error_t tlsComputeMac(TlsContext *context, TlsEncryptionEngine *encryptionEngine,
   const void *record, const uint8_t *data, size_t dataLen, uint8_t *mac);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
