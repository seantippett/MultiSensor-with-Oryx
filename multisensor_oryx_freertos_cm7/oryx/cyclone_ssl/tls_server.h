/**
 * @file tls_server.h
 * @brief Handshake message processing (TLS server)
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

#ifndef _TLS_SERVER_H
#define _TLS_SERVER_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS server specific functions
error_t tlsSendServerHello(TlsContext *context);
error_t tlsSendServerKeyExchange(TlsContext *context);
error_t tlsSendCertificateRequest(TlsContext *context);
error_t tlsSendServerHelloDone(TlsContext *context);
error_t tlsSendNewSessionTicket(TlsContext *context);

error_t tlsFormatServerHello(TlsContext *context,
   TlsServerHello *message, size_t *length);

error_t tlsFormatServerKeyExchange(TlsContext *context,
   TlsServerKeyExchange *message, size_t *length);

error_t tlsFormatCertificateRequest(TlsContext *context,
   TlsCertificateRequest *message, size_t *length);

error_t tlsFormatServerHelloDone(TlsContext *context,
   TlsServerHelloDone *message, size_t *length);

error_t tlsFormatNewSessionTicket(TlsContext *context,
   TlsNewSessionTicket *message, size_t *length);

error_t tlsParseClientHello(TlsContext *context,
   const TlsClientHello *message, size_t length);

error_t tlsParseClientKeyExchange(TlsContext *context,
   const TlsClientKeyExchange *message, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
