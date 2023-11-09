/**
 * @file tls_client.h
 * @brief Handshake message processing (TLS client)
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

#ifndef _TLS_CLIENT_H
#define _TLS_CLIENT_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS client specific functions
error_t tlsSendClientHello(TlsContext *context);
error_t tlsSendClientKeyExchange(TlsContext *context);

error_t tlsFormatClientHello(TlsContext *context,
   TlsClientHello *message, size_t *length);

error_t tlsFormatClientKeyExchange(TlsContext *context,
   TlsClientKeyExchange *message, size_t *length);

error_t tlsParseHelloRequest(TlsContext *context,
   const TlsHelloRequest *message, size_t length);

error_t tlsParseServerHello(TlsContext *context,
   const TlsServerHello *message, size_t length);

error_t tlsParseServerKeyExchange(TlsContext *context,
   const TlsServerKeyExchange *message, size_t length);

error_t tlsParseCertificateRequest(TlsContext *context,
   const TlsCertificateRequest *message, size_t length);

error_t tlsParseServerHelloDone(TlsContext *context,
   const TlsServerHelloDone *message, size_t length);

error_t tlsParseNewSessionTicket(TlsContext *context,
   const TlsNewSessionTicket *message, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
