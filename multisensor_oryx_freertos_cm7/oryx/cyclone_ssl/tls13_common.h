/**
 * @file tls13_common.h
 * @brief Handshake message processing (TLS 1.3 client and server)
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

#ifndef _TLS13_COMMON_H
#define _TLS13_COMMON_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS 1.3 related functions
error_t tls13SendKeyUpdate(TlsContext *context);

error_t tls13FormatKeyUpdate(TlsContext *context, Tls13KeyUpdate *message,
   size_t *length);

error_t tls13ParseKeyUpdate(TlsContext *context, const Tls13KeyUpdate *message,
   size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
