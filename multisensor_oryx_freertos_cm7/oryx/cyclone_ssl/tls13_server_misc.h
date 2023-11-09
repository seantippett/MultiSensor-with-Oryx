/**
 * @file tls13_server_misc.h
 * @brief Helper functions for TLS 1.3 server
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

#ifndef _TLS13_SERVER_MISC_H
#define _TLS13_SERVER_MISC_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS 1.3 server specific functions
error_t tls13NegotiateCipherSuite(TlsContext *context, const void *clientHello,
   size_t clientHelloLen, const TlsCipherSuites *cipherSuites,
   TlsHelloExtensions *extensions);

error_t tls13SelectGroup(TlsContext *context,
   const TlsSupportedGroupList *groupList);

error_t tls13VerifyPskBinder(TlsContext *context, const void *clientHello,
   size_t clientHelloLen, const Tls13PskIdentityList *identityList,
   const Tls13PskBinderList *binderList, int_t selectedIdentity);

error_t tls13ProcessEarlyData(TlsContext *context, const uint8_t *data,
   size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
