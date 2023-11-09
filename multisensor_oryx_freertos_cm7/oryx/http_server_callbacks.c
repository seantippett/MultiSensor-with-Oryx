/**
 * @file http_server_callbacks.c
 * @brief Network initialization
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

//Dependencies
#include "core/net.h"
#include "http_server_callbacks.h"
#include "rng/yarrow.h"
#include "debug.h"

//HTTPS server configuration
#define APP_HTTP_SERVER_CERT "certs/server_cert.pem"
#define APP_HTTP_SERVER_KEY "certs/server_key.pem"

//Global variables
extern TlsCache *tlsCache;
extern YarrowContext yarrowContext;

//Forward declaration of functions
error_t httpServerProcessTestGet(HttpConnection *connection,
   const char_t *uri);

error_t httpServerProcessTestPut(HttpConnection *connection,
   const char_t *uri);

error_t httpServerProcessTestPost(HttpConnection *connection,
   const char_t *uri);


/**
 * @brief TLS initialization callback
 * @param[in] connection Handle referencing a client connection
 * @param[in] tlsContext Pointer to the TLS context
 * @return Error code
 **/

error_t httpServerTlsInitCallback(HttpConnection *connection,
   TlsContext *tlsContext)
{
   error_t error;
   const char_t *serverCert;
   size_t serverCertLen;
   const char_t *serverKey;
   size_t serverKeyLen;

   //Set TX and RX buffer size
   error = tlsSetBufferSize(tlsContext, 2048, 16384);
   //Any error to report?
   if(error)
      return error;

   //Set the PRNG algorithm to be used
   error = tlsSetPrng(tlsContext, YARROW_PRNG_ALGO, &yarrowContext);
   //Any error to report?
   if(error)
      return error;

   //Session cache that will be used to save/resume TLS sessions
   error = tlsSetCache(tlsContext, tlsCache);
   //Any error to report?
   if(error)
      return error;

   //Client authentication is not required
   error = tlsSetClientAuthMode(tlsContext, TLS_CLIENT_AUTH_NONE);
   //Any error to report?
   if(error)
      return error;

   //Get server's certificate
   error = resGetData(APP_HTTP_SERVER_CERT, (const uint8_t **) &serverCert,
      &serverCertLen);
   //Any error to report?
   if(error)
      return error;

   //Get server's private key
   error = resGetData(APP_HTTP_SERVER_KEY, (const uint8_t **) &serverKey,
      &serverKeyLen);
   //Any error to report?
   if(error)
      return error;

   //Load server's certificate
   error = tlsLoadCertificate(tlsContext, 0, serverCert, serverCertLen,
      serverKey, serverKeyLen, NULL);
   //Any error to report?
   if(error)
      return error;

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief CGI callback function
 * @param[in] connection Handle referencing a client connection
 * @param[in] param NULL-terminated string that contains the CGI parameter
 * @return Error code
 **/

error_t httpServerCgiCallback(HttpConnection *connection,
   const char_t *param)
{
   static uint_t pageCounter = 0;
   uint_t length;
   MacAddr macAddr;
#if (IPV4_SUPPORT == ENABLED)
   Ipv4Addr ipv4Addr;
#endif
#if (IPV6_SUPPORT == ENABLED)
   uint_t n;
   Ipv6Addr ipv6Addr;
#endif

   //Underlying network interface
   NetInterface *interface = connection->socket->interface;

   //Check parameter name
   if(!strcasecmp(param, "PAGE_COUNTER"))
   {
      pageCounter++;
      sprintf(connection->buffer, "%u time%s", pageCounter, (pageCounter >= 2) ? "s" : "");
   }
   else if(!strcasecmp(param, "BOARD_NAME"))
   {
      strcpy(connection->buffer, "SENSTAR Multisensor");
   }
   else if(!strcasecmp(param, "SYSTEM_TIME"))
   {
      systime_t time = osGetSystemTime();
      formatSystemTime(time, connection->buffer);
   }
   else if(!strcasecmp(param, "MAC_ADDR"))
   {
      netGetMacAddr(interface, &macAddr);
      macAddrToString(&macAddr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV4_ADDR"))
   {
      ipv4GetHostAddr(interface, &ipv4Addr);
      ipv4AddrToString(ipv4Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "SUBNET_MASK"))
   {
      ipv4GetSubnetMask(interface, &ipv4Addr);
      ipv4AddrToString(ipv4Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "DEFAULT_GATEWAY"))
   {
      ipv4GetDefaultGateway(interface, &ipv4Addr);
      ipv4AddrToString(ipv4Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV4_PRIMARY_DNS"))
   {
      ipv4GetDnsServer(interface, 0, &ipv4Addr);
      ipv4AddrToString(ipv4Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV4_SECONDARY_DNS"))
   {
      ipv4GetDnsServer(interface, 1, &ipv4Addr);
      ipv4AddrToString(ipv4Addr, connection->buffer);
   }
#if (IPV6_SUPPORT == ENABLED)
   else if(!strcasecmp(param, "LINK_LOCAL_ADDR"))
   {
      ipv6GetLinkLocalAddr(interface, &ipv6Addr);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "GLOBAL_ADDR"))
   {
      ipv6GetGlobalAddr(interface, 0, &ipv6Addr);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV6_PREFIX"))
   {
      ipv6GetPrefix(interface, 0, &ipv6Addr, &n);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
      length = strlen(connection->buffer);
      sprintf(connection->buffer + length, "/%u", n);
   }
   else if(!strcasecmp(param, "ROUTER"))
   {
      ipv6GetDefaultRouter(interface, 0, &ipv6Addr);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV6_PRIMARY_DNS"))
   {
      ipv6GetDnsServer(interface, 0, &ipv6Addr);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
   }
   else if(!strcasecmp(param, "IPV6_SECONDARY_DNS"))
   {
      ipv6GetDnsServer(interface, 1, &ipv6Addr);
      ipv6AddrToString(&ipv6Addr, connection->buffer);
   }
#endif
   else
   {
      return ERROR_INVALID_TAG;
   }

   //Get the length of the resulting string
   length = strlen(connection->buffer);

   //Send the contents of the specified environment variable
   return httpWriteStream(connection, connection->buffer, length);
}


/**
 * @brief URI not found callback
 * @param[in] connection Handle referencing a client connection
 * @param[in] uri NULL-terminated string containing the path to the requested resource
 * @return Error code
 **/

error_t httpServerUriNotFoundCallback(HttpConnection *connection,
   const char_t *uri)
{
   //Not implemented
   return ERROR_NOT_FOUND;
}


/**
 * @brief HTTP request callback
 * @param[in] connection Handle referencing a client connection
 * @param[in] uri NULL-terminated string containing the path to the requested resource
 * @return Error code
 **/

error_t httpServerRequestCallback(HttpConnection *connection,
   const char_t *uri)
{
   error_t error;

   //Check HTTP method name
   if(!strcasecmp(connection->request.method, "GET"))
   {
      //Check URI
      if(!strcasecmp(uri, "/test-get"))
      {
         //Process GET request
         error = httpServerProcessTestGet(connection, uri);
      }
      else
      {
         //Unknown URI
         error = ERROR_NOT_FOUND;
      }
   }
   else if(!strcasecmp(connection->request.method, "PUT"))
   {
      //Check URI
      if(!strcasecmp(uri, "/test-put"))
      {
         //Process GET request
         error = httpServerProcessTestPut(connection, uri);
      }
      else
      {
         //Unknown URI
         error = ERROR_NOT_FOUND;
      }
   }
   else if(!strcasecmp(connection->request.method, "POST"))
   {
      //Check URI
      if(!strcasecmp(uri, "/test-post"))
      {
         //Process POST request
         error = httpServerProcessTestPost(connection, uri);
      }
      else
      {
         //Unknown URI
         error = ERROR_NOT_FOUND;
      }
   }
   else
   {
      //Invalid HTTP method
      error = ERROR_NOT_FOUND;
   }

   //Return status code
   return error;
}


/**
 * @brief HTTP GET request processing example
 * @param[in] connection Handle referencing a client connection
 * @param[in] uri NULL-terminated string containing the path to the requested resource
 * @return Error code
 **/

error_t httpServerProcessTestGet(HttpConnection *connection,
   const char_t *uri)
{
   error_t error;
   size_t n;
   char_t buffer[128];
   static uint_t counter;

   //Format HTTP response header
   connection->response.version = connection->request.version;
   connection->response.statusCode = 200;
   connection->response.keepAlive = connection->request.keepAlive;
   connection->response.noCache = TRUE;
   connection->response.contentType = mimeGetType(".json");

   //The size of the response body is unknown. The response will be sent in
   //multiple chunks
   connection->response.chunkedEncoding = TRUE;
   connection->response.contentLength = 0;

   //Send the header to the client
   error = httpWriteHeader(connection);
   //Any error to report?
   if(error)
      return error;

   //Generate dynamic contents (1st fragment)
   n = sprintf(buffer, "{\"counter\": %u, ", ++counter);
   //Send data over the HTTP connection
   error = httpWriteStream(connection, buffer, n);
   //Any error to report?
   if(error)
      return error;

   //Generate dynamic contents (2nd fragment)
   n = sprintf(buffer, "\"message\": \"%s\"", "This is a response from test-get!");
   //Send data over the HTTP connection
   error = httpWriteStream(connection, buffer, n);
   //Any error to report?
   if(error)
      return error;

   //Generate dynamic contents (3rd fragment)
   n = sprintf(buffer, "}");
   //Send data over the HTTP connection
   error = httpWriteStream(connection, buffer, n);
   //Any error to report?
   if(error)
      return error;

   //Properly close output stream
   error = httpCloseStream(connection);
   //Return status code
   return error;
}


/**
 * @brief HTTP PUT request processing example
 * @param[in] connection Handle referencing a client connection
 * @param[in] uri NULL-terminated string containing the path to the requested resource
 * @return Error code
 **/


error_t httpServerProcessTestPut(HttpConnection *connection,
   const char_t *uri)
{
   error_t error;
   size_t n;
   char_t buffer[128];

   //Initialize status code
   error = NO_ERROR;

   //Debug message
   TRACE_INFO("HTTP request body:\r\n");

   //Consume HTTP request body
   while(!error)
   {
      //Read HTTP request body
      error = httpReadStream(connection, buffer, sizeof(buffer) - 1, &n, 0);

      //Check status code
      if(!error)
      {
         //Process data here...
         buffer[n] = '\0';
         TRACE_INFO("%s", buffer);
      }
   }

   //Make sure the entire request body has been received
   if(error != ERROR_END_OF_STREAM)
      return error;

   //Debug message
   TRACE_INFO("\r\n\r\n");

   //Format HTTP response header
   connection->response.version = connection->request.version;
   connection->response.statusCode = 200;
   connection->response.keepAlive = connection->request.keepAlive;
   connection->response.noCache = TRUE;
   connection->response.contentType = NULL;
   connection->response.chunkedEncoding = FALSE;
   connection->response.contentLength = 0;

   //Send the header to the client
   error = httpWriteHeader(connection);
   //Any error to report?
   if(error)
      return error;

   //Properly close output stream
   error = httpCloseStream(connection);
   //Return status code
   return error;
}


/**
 * @brief HTTP POST request processing example
 * @param[in] connection Handle referencing a client connection
 * @param[in] uri NULL-terminated string containing the path to the requested resource
 * @return Error code
 **/


error_t httpServerProcessTestPost(HttpConnection *connection,
   const char_t *uri)
{
   error_t error;
   size_t n;
   char_t buffer[128];

   //Initialize status code
   error = NO_ERROR;

   //Debug message
   TRACE_INFO("HTTP request body:\r\n");

   //Consume HTTP request body
   while(!error)
   {
      //Read HTTP request body
      error = httpReadStream(connection, buffer, sizeof(buffer) - 1, &n, 0);

      //Check status code
      if(!error)
      {
         //Process data here...
         buffer[n] = '\0';
         TRACE_INFO("%s", buffer);
      }
   }

   //Make sure the entire request body has been received
   if(error != ERROR_END_OF_STREAM)
      return error;

   //Debug message
   TRACE_INFO("\r\n\r\n");

   //Format response body
   n = sprintf(buffer, "This is a response from test-post!\r\n");

   //Format HTTP response header
   connection->response.version = connection->request.version;
   connection->response.statusCode = 200;
   connection->response.keepAlive = connection->request.keepAlive;
   connection->response.noCache = TRUE;
   connection->response.contentType = mimeGetType(".txt");

   //The size of the response body is known in advance
   connection->response.chunkedEncoding = FALSE;
   connection->response.contentLength = n;

   //Send the response header to the client
   error = httpWriteHeader(connection);
   //Any error to report?
   if(error)
      return error;

   //Send the response body to the client
   error = httpWriteStream(connection, buffer, n);
   //Any error to report?
   if(error)
      return error;

   //Properly close output stream
   error = httpCloseStream(connection);
   //Return status code
   return error;
}
