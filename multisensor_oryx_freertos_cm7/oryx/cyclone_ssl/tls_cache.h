/**
 * @file tls_cache.h
 * @brief Session cache management
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

#ifndef _TLS_CACHE_H
#define _TLS_CACHE_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Session cache management
TlsCache *tlsInitCache(uint_t size);

TlsSessionState *tlsFindCache(TlsCache *cache, const uint8_t *sessionId,
   size_t sessionIdLen);

error_t tlsSaveToCache(TlsContext *context);
error_t tlsRemoveFromCache(TlsContext *context);
void tlsFreeCache(TlsCache *cache);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
