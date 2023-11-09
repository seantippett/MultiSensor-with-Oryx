/**
 * @file tls_legacy.h
 * @brief Legacy definitions
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

#ifndef _TLS_LEGACY_H
#define _TLS_LEGACY_H

//Deprecated definitions
#define TlsIoHandle TlsSocketHandle

//Deprecated functions
#define tlsSetIoCallbacks(context, handle, sendCallback, receiveCallback) \
   tlsSetSocketCallbacks(context, sendCallback, receiveCallback, handle);

#ifdef TLS_RSA_SUPPORT
   #if (TLS_RSA_SUPPORT == ENABLED)
      #define TLS_RSA_KE_SUPPORT ENABLED
   #else
      #define TLS_RSA_KE_SUPPORT DISABLED
   #endif
   #undef TLS_RSA_SUPPORT
#endif

#ifdef TLS_DHE_RSA_SUPPORT
   #define TLS_DHE_RSA_KE_SUPPORT TLS_DHE_RSA_SUPPORT
#endif

#ifdef TLS_DHE_DSS_SUPPORT
   #define TLS_DHE_DSS_KE_SUPPORT TLS_DHE_DSS_SUPPORT
#endif

#ifdef TLS_DH_ANON_SUPPORT
   #define TLS_DH_ANON_KE_SUPPORT TLS_DH_ANON_SUPPORT
#endif

#ifdef TLS_ECDHE_RSA_SUPPORT
   #define TLS_ECDHE_RSA_KE_SUPPORT TLS_ECDHE_RSA_SUPPORT
#endif

#ifdef TLS_ECDHE_ECDSA_SUPPORT
   #define TLS_ECDHE_ECDSA_KE_SUPPORT TLS_ECDHE_ECDSA_SUPPORT
#endif

#ifdef TLS_ECDH_ANON_SUPPORT
   #define TLS_ECDH_ANON_KE_SUPPORT TLS_ECDH_ANON_SUPPORT
#endif

#ifdef TLS_PSK_SUPPORT
   #if (TLS_PSK_SUPPORT == ENABLED)
      #define TLS_PSK_KE_SUPPORT ENABLED
   #else
      #define TLS_PSK_KE_SUPPORT DISABLED
   #endif
   #undef TLS_PSK_SUPPORT
#endif

#ifdef TLS_RSA_PSK_SUPPORT
   #define TLS_RSA_PSK_KE_SUPPORT TLS_RSA_PSK_SUPPORT
#endif

#ifdef TLS_DHE_PSK_SUPPORT
   #define TLS_DHE_PSK_KE_SUPPORT TLS_DHE_PSK_SUPPORT
#endif

#ifdef TLS_ECDHE_PSK_SUPPORT
   #define TLS_ECDHE_PSK_KE_SUPPORT TLS_ECDHE_PSK_SUPPORT
#endif

#ifdef TLS_CURVE25519_SUPPORT
   #define TLS_X25519_SUPPORT TLS_CURVE25519_SUPPORT
#endif

#ifdef TLS_CURVE448_SUPPORT
   #define TLS_X448_SUPPORT TLS_CURVE448_SUPPORT
#endif

#define TlsSession TlsSessionState
#define tlsSaveSession tlsSaveSessionState
#define tlsRestoreSession tlsRestoreSessionState

#ifdef TLS_AES_SUPPORT
   #define TLS_AES_128_SUPPORT TLS_AES_SUPPORT
   #define TLS_AES_256_SUPPORT TLS_AES_SUPPORT
#endif

#ifdef TLS_CAMELLIA_SUPPORT
   #define TLS_CAMELLIA_128_SUPPORT TLS_CAMELLIA_SUPPORT
   #define TLS_CAMELLIA_256_SUPPORT TLS_CAMELLIA_SUPPORT
#endif

#ifdef TLS_ARIA_SUPPORT
   #define TLS_ARIA_128_SUPPORT TLS_ARIA_SUPPORT
   #define TLS_ARIA_256_SUPPORT TLS_ARIA_SUPPORT
#endif

#endif
