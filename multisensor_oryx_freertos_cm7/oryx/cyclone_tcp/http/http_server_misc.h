/**
 * @file http_server_misc.h
 * @brief HTTP server (miscellaneous functions)
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

#ifndef _HTTP_SERVER_MISC_H
#define _HTTP_SERVER_MISC_H

//Dependencies
#include "http/http_server.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//HTTP server related functions
error_t httpReadRequestHeader(HttpConnection *connection);
error_t httpParseRequestLine(HttpConnection *connection, char_t *requestLine);

error_t httpReadHeaderField(HttpConnection *connection,
   char_t *buffer, size_t size, char_t *firstChar);

void httpParseHeaderField(HttpConnection *connection,
   const char_t *name, char_t *value);

void httpParseConnectionField(HttpConnection *connection,
   char_t *value);

void httpParseContentTypeField(HttpConnection *connection,
   char_t *value);

void httpParseAcceptEncodingField(HttpConnection *connection,
   char_t *value);

void httpParseCookieField(HttpConnection *connection, char_t *value);

error_t httpReadChunkSize(HttpConnection *connection);

void httpInitResponseHeader(HttpConnection *connection);
error_t httpFormatResponseHeader(HttpConnection *connection, char_t *buffer);

error_t httpSend(HttpConnection *connection,
   const void *data, size_t length, uint_t flags);

error_t httpReceive(HttpConnection *connection,
   void *data, size_t size, size_t *received, uint_t flags);

void httpGetAbsolutePath(HttpConnection *connection,
   const char_t *relative, char_t *absolute, size_t maxLen);

bool_t httpCompExtension(const char_t *filename, const char_t *extension);

error_t httpDecodePercentEncodedString(const char_t *input,
   char_t *output, size_t outputSize);

void httpConvertArrayToHexString(const uint8_t *input,
   size_t inputLen, char_t *output);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
