/**
 * @file http_server_callbacks.c
 * @brief Network initialization
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _HTTP_SERVER_CALLBACKS
#define _HTTP_SERVER_CALLBACKS

//Dependencies
#include "http/http_server.h"

//HTTP server callbacks
error_t httpServerTlsInitCallback(HttpConnection *connection,
   TlsContext *tlsContext);

error_t httpServerCgiCallback(HttpConnection *connection,
   const char_t *param);

error_t httpServerUriNotFoundCallback(HttpConnection *connection,
   const char_t *uri);

error_t httpServerRequestCallback(HttpConnection *connection,
   const char_t *uri);
 
#endif
