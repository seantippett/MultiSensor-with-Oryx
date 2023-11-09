/**
 * @file http_server_auth.h
 * @brief HTTP authentication
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Eval.
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

#ifndef _HTTP_SERVER_AUTH_H
#define _HTTP_SERVER_AUTH_H

//Dependencies
#include "http/http_server.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//HTTP authentication related functions
bool_t httpCheckPassword(HttpConnection *connection,
   const char_t *password, HttpAuthMode mode);

void httpParseAuthorizationField(HttpConnection *connection, char_t *value);
size_t httpAddAuthenticateField(HttpConnection *connection, char_t *output);

error_t httpGenerateNonce(HttpServerContext *context,
   char_t *output, size_t *length);

error_t httpVerifyNonce(HttpServerContext *context,
   const char_t *nonce, const char_t *nc);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
