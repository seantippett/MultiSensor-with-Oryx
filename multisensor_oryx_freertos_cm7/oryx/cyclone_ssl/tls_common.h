/**
 * @file tls_common.h
 * @brief Handshake message processing (TLS client and server)
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

#ifndef _TLS_COMMON_H
#define _TLS_COMMON_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsSendCertificate(TlsContext *context);
error_t tlsSendCertificateVerify(TlsContext *context);
error_t tlsSendChangeCipherSpec(TlsContext *context);
error_t tlsSendFinished(TlsContext *context);
error_t tlsSendAlert(TlsContext *context, uint8_t level, uint8_t description);

error_t tlsFormatCertificate(TlsContext *context,
   TlsCertificate *message, size_t *length);

error_t tlsFormatCertificateVerify(TlsContext *context,
   TlsCertificateVerify *message, size_t *length);

error_t tlsFormatChangeCipherSpec(TlsContext *context,
   TlsChangeCipherSpec *message, size_t *length);

error_t tlsFormatFinished(TlsContext *context,
   TlsFinished *message, size_t *length);

error_t tlsFormatAlert(TlsContext *context, uint8_t level,
   uint8_t description, TlsAlert *message, size_t *length);

error_t tlsFormatSignAlgosExtension(TlsContext *context,
   uint_t cipherSuiteTypes, uint8_t *p, size_t *written);

error_t tlsFormatSignAlgosCertExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatCertAuthoritiesExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatCertAuthorities(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsParseCertificate(TlsContext *context,
   const TlsCertificate *message, size_t length);

error_t tlsParseCertificateVerify(TlsContext *context,
   const TlsCertificateVerify *message, size_t length);

error_t tlsParseChangeCipherSpec(TlsContext *context,
   const TlsChangeCipherSpec *message, size_t length);

error_t tlsParseFinished(TlsContext *context,
   const TlsFinished *message, size_t length);

error_t tlsParseAlert(TlsContext *context,
   const TlsAlert *message, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
