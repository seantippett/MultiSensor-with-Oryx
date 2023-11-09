//Dependencies
#include "core/net.h"
#include "tls.h"
#include "tls_cipher_suites.h"
#include "tls_ticket.h"
#include "tls_misc.h"
#include "rng/yarrow.h"
#include "resource_manager.h"
#include "debug.h"

//Application configuration
#define APP_SERVER_PORT 4433
#define APP_SERVER_MAX_CONNECTIONS 2
#define APP_SERVER_TIMEOUT 15000
#define APP_SERVER_CERT "certs/server_cert.pem"
#define APP_SERVER_KEY "certs/server_key.pem"
#define APP_CA_CERT "certs/ca_cert.pem"

//Client's PSK identity
#define APP_CLIENT1_PSK_IDENTITY "Client1"
#define APP_CLIENT2_PSK_IDENTITY "Client2"

//Client's PSK
const uint8_t client1Psk[] = {
   0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
   0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

const uint8_t client2Psk[] = {
   0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
   0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};

//List of preferred ciphersuites
const uint16_t cipherSuites[] =
{
   TLS_CHACHA20_POLY1305_SHA256,
   TLS_AES_128_GCM_SHA256
};

//Global variables
static uint_t hitCounter = 0;

static OsSemaphore connectionSemaphore;
static TlsTicketContext tlsTicketContext;

extern YarrowContext yarrowContext;

//Forward declaration of functions
void tlsServerDemoTask(void *param);
void tlsClientDemoTask(void *param);

error_t tlsServerPskCallback(TlsContext *context, const uint8_t *pskIdentity,
   size_t pskIdentityLen);

size_t dumpArray(char_t *buffer, const uint8_t *data, size_t length);


/**
 * @brief TLS server initialization
 **/

void tlsServerDemoInit(void)
{
   portBASE_TYPE status;
   TaskHandle_t handle;

   //Create a task to handle incoming requests
   status = xTaskCreate(tlsServerDemoTask, "TLS Server", 500, NULL,
      tskIDLE_PRIORITY + 1, &handle);

   //Failed to create the task?
   if(status != pdPASS)
   {
      //Debug message
      TRACE_ERROR("Failed to create task!\r\n");
   }
}


/**
 * @brief TLS server task
 * @param param[in] Not used
 **/

void tlsServerDemoTask(void *param)
{
   error_t error;
   uint_t counter;
   uint16_t clientPort;
   IpAddr clientIpAddr;
   Socket *serverSocket;
   Socket *clientSocket;
   portBASE_TYPE status;
   TaskHandle_t handle;

   //Create a semaphore to limit the number of simultaneous connections
   if(!osCreateSemaphore(&connectionSemaphore, APP_SERVER_MAX_CONNECTIONS))
   {
      //Debug message
      TRACE_ERROR("Failed to create semaphore!\r\n");
   }

#if (TLS_TICKET_SUPPORT == ENABLED)
   //Initialize ticket encryption context
   error = tlsInitTicketContext(&tlsTicketContext);
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to bind socket!\r\n");
   }
#endif

   //Open a socket
   serverSocket = socketOpen(SOCKET_TYPE_STREAM, SOCKET_IP_PROTO_TCP);
   //Failed to open socket?
   if(!serverSocket)
   {
      //Debug message
      TRACE_ERROR("Cannot open socket!\r\n");
   }

   //Bind newly created socket to port 443
   error = socketBind(serverSocket, &IP_ADDR_ANY, APP_SERVER_PORT);
   //Failed to bind socket to port 443?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to bind socket!\r\n");
   }

   //Place socket in listening state
   error = socketListen(serverSocket, 1);
   //Any failure to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to enter listening state!\r\n");
   }

   //Process incoming connections to the server
   for(counter = 1; ; counter++)
   {
      //Debug message
      TRACE_INFO("\r\n\r\n");
      TRACE_INFO("Waiting for an incoming connection...\r\n\r\n");

      //Limit the number of simultaneous connections to the HTTP server
      osWaitForSemaphore(&connectionSemaphore, INFINITE_DELAY);

      //Accept an incoming connection
      clientSocket = socketAccept(serverSocket, &clientIpAddr, &clientPort);

      //Make sure the socket handle is valid
      if(clientSocket != NULL)
      {
         //Debug message
         TRACE_INFO("Connection #%u established with client %s port %" PRIu16 "...\r\n",
            counter, ipAddrToString(&clientIpAddr, NULL), clientPort);

         //Create a task to service the client connection
         status = xTaskCreate(tlsClientDemoTask, "TLS Client", 800, clientSocket,
            tskIDLE_PRIORITY + 1, &handle);

         //Did we encounter an error?
         if(status != pdPASS)
         {
            //Debug message
            TRACE_ERROR("Failed to create task!\r\n");

            //Close socket
            socketClose(clientSocket);
            //Release semaphore
            osReleaseSemaphore(&connectionSemaphore);
         }
      }
   }
}


/**
 * @brief TLS client task
 * @param param[in] Client socket
 **/

void tlsClientDemoTask(void *param)
{
   error_t error;
   size_t n;
   Socket *clientSocket;
   TlsContext *tlsContext;
   char_t buffer[512];

   //Variable initialization
   tlsContext = NULL;

   //Retrieve socket handle
   clientSocket = (Socket *) param;

   //Start of exception handling block
   do
   {
      //Set timeout
      error = socketSetTimeout(clientSocket, APP_SERVER_TIMEOUT);
      //Any error to report?
      if(error)
         break;

      //TLS context initialization
      tlsContext = tlsInit();
      //Failed to initialize TLS context?
      if(tlsContext == NULL)
      {
         //Report an error
         error = ERROR_OUT_OF_MEMORY;
         //Exit immediately
         break;
      }

      //Select server operation mode
      error = tlsSetConnectionEnd(tlsContext, TLS_CONNECTION_END_SERVER);
      //Any error to report?
      if(error)
         break;

      //Bind TLS to the relevant socket
      error = tlsSetSocket(tlsContext, clientSocket);
      //Any error to report?
      if(error)
         break;

      //Set TX and RX buffer size
      error = tlsSetBufferSize(tlsContext, 2048, 16384);
      //Any error to report?
      if(error)
         break;

      //Set the PRNG algorithm to be used
      error = tlsSetPrng(tlsContext, YARROW_PRNG_ALGO, &yarrowContext);
      //Any error to report?
      if(error)
         break;

      //Set supported TLS version(s)
      error = tlsSetVersion(tlsContext, TLS_VERSION_1_3, TLS_VERSION_1_3);
      //Any error to report?
      if(error)
         break;

      //Preferred cipher suite list
      error = tlsSetCipherSuites(tlsContext, cipherSuites,
         arraysize(cipherSuites));
      //Any error to report?
      if(error)
         break;

      //Register PSK callback function
      error = tlsSetPskCallback(tlsContext, tlsServerPskCallback);
      //Any error to report?
      if(error)
         break;

#if (TLS_TICKET_SUPPORT == ENABLED)
      //Enable session ticket mechanism
      error = tlsEnableSessionTickets(tlsContext, TRUE);
      //Any error to report?
      if(error)
         break;

      //Enable session ticket mechanism
      error = tlsSetTicketCallbacks(tlsContext, tlsEncryptTicket,
         tlsDecryptTicket, &tlsTicketContext);
      //Any error to report?
      if(error)
         break;
#endif

      //Establish a secure session
      error = tlsConnect(tlsContext);
      //TLS handshake failure?
      if(error)
         break;

      //Debug message
      TRACE_INFO("\r\n");
      TRACE_INFO("HTTP request:\r\n");

      //Read HTTP request
      while(1)
      {
         //Read a complete line
         error = tlsRead(tlsContext, buffer, sizeof(buffer) - 1, &n,
            TLS_FLAG_BREAK_CRLF);
         //Any error to report?
         if(error)
            break;

         //Properly terminate the string with a NULL character
         buffer[n] = '\0';
         //Dump HTTP request
         TRACE_INFO("%s", buffer);

         //The end of the header has been reached?
         if(!strcmp(buffer, "\r\n"))
            break;
      }

      //Propagate exception if necessary
      if(error)
         break;

      //Debug message
      TRACE_INFO("HTTP response:\r\n");

      //Format response
      n = 0;
      n += sprintf(buffer + n, "HTTP/1.0 200 OK\r\n");
      n += sprintf(buffer + n, "Content-Type: text/html\r\n");
      n += sprintf(buffer + n, "\r\n");

      n += sprintf(buffer + n, "<!DOCTYPE html>\r\n");
      n += sprintf(buffer + n, "<html>\r\n");
      n += sprintf(buffer + n, "<head>\r\n");
      n += sprintf(buffer + n, "  <title>Oryx Embedded - CycloneSSL TLS Server Demo</title>\r\n");
      n += sprintf(buffer + n, "  <style>\r\n");
      n += sprintf(buffer + n, "    body {font-family: monospace; font-size: 13px;}\r\n");
      n += sprintf(buffer + n, "    table {border-width: 1px; border-style: ouset; border-collapse: collapse;}\r\n");
      n += sprintf(buffer + n, "    td {border-width: 1px; border-style: inset; padding: 3px;}\r\n");
      n += sprintf(buffer + n, "  </style>\r\n");
      n += sprintf(buffer + n, "</head>\r\n");
      n += sprintf(buffer + n, "<body>\r\n");
      n += sprintf(buffer + n, "  <p>Welcome to the CycloneSSL TLS server!</p>\r\n");
      n += sprintf(buffer + n, "  <table>\r\n");

      //Debug message
      TRACE_INFO("%s\r\n", buffer);

      //Send response to the client
      error = tlsWrite(tlsContext, buffer, n, NULL, 0);
      //Any error to report?
      if(error)
         break;

      //Format response
      n = 0;
      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Hit counter</td>\r\n");
      n += sprintf(buffer + n, "    <td>%d</td>\r\n", ++hitCounter);
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Server version</td>\r\n");
      n += sprintf(buffer + n, "    <td>%s</td>\r\n",
         tlsGetVersionName(tlsContext->versionMax));
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Client version</td>\r\n");
      n += sprintf(buffer + n, "    <td>%s</td>\r\n",
         tlsGetVersionName(MAX(tlsContext->clientVersion, tlsContext->version)));
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Negotiated version</td>\r\n");
      n += sprintf(buffer + n, "    <td>%s</td>\r\n",
         tlsGetVersionName(tlsContext->version));
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Cipher suite</td>\r\n");
      n += sprintf(buffer + n, "    <td>%s</td>\r\n",
         tlsGetCipherSuiteName(tlsContext->cipherSuite.identifier));
      n += sprintf(buffer + n, "  </tr>\r\n");

      //Debug message
      TRACE_INFO("%s\r\n", buffer);

      //Send response to the client
      error = tlsWrite(tlsContext, buffer, n, NULL, 0);
      //Any error to report?
      if(error)
         break;

      //Format response
      n = 0;
      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Client random</td>\r\n");
      n += sprintf(buffer + n, "      <td>\r\n");
      n += sprintf(buffer + n, "        ");
      n += dumpArray(buffer + n, (uint8_t *) &tlsContext->clientRandom, 32);
      n += sprintf(buffer + n, "\r\n");
      n += sprintf(buffer + n, "      </td>\r\n");
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Server random</td>\r\n");
      n += sprintf(buffer + n, "      <td>\r\n");
      n += sprintf(buffer + n, "        ");
      n += dumpArray(buffer + n, (uint8_t *) &tlsContext->serverRandom, 32);
      n += sprintf(buffer + n, "\r\n");
      n += sprintf(buffer + n, "      </td>\r\n");
      n += sprintf(buffer + n, "  </tr>\r\n");

      //Debug message
      TRACE_INFO("%s\r\n", buffer);

      //Send response to the client
      error = tlsWrite(tlsContext, buffer, n, NULL, 0);
      //Any error to report?
      if(error)
         break;

      //Format response
      n = 0;
      n += sprintf(buffer + n, "  <tr>\r\n");
      n += sprintf(buffer + n, "    <td>Session ID</td>\r\n");
      n += sprintf(buffer + n, "      <td>\r\n");
      n += sprintf(buffer + n, "        ");
      n += dumpArray(buffer + n, tlsContext->sessionId, tlsContext->sessionIdLen);
      n += sprintf(buffer + n, "\r\n");
      n += sprintf(buffer + n, "      </td>\r\n");
      n += sprintf(buffer + n, "  </tr>\r\n");

      n += sprintf(buffer + n, "  </table>\r\n");
      n += sprintf(buffer + n, "</body>\r\n");
      n += sprintf(buffer + n, "</html>\r\n");

      //Debug message
      TRACE_INFO("%s\r\n", buffer);

      //Send response to the client
      error = tlsWrite(tlsContext, buffer, n, NULL, 0);
      //Any error to report?
      if(error)
         break;

      //Terminate TLS session
      error = tlsShutdown(tlsContext);
      //Any error to report?
      if(error)
         break;

      //Graceful shutdown
      error = socketShutdown(clientSocket, SOCKET_SD_BOTH);
      //Any error to report?
      if(error)
         break;

      //End of exception handling block
   } while(0);

   //Release TLS context
   if(tlsContext != NULL)
   {
      tlsFree(tlsContext);
   }

   //Close socket
   if(clientSocket != NULL)
   {
      socketClose(clientSocket);
   }

   //Debug message
   TRACE_INFO("Connection closed...\r\n");

   //Release semaphore
   osReleaseSemaphore(&connectionSemaphore);

   //Kill ourselves
   osDeleteTask(OS_SELF_TASK_ID);
}


/**
 * @brief PSK callback function
 * @param[in] context Pointer to the TLS context
 * @param[in] pskIdentity PSK identity of the client
 * @param[in] pskIdentityLen Length of the PSK identity, in bytes
 * @return Error code
 **/

error_t tlsServerPskCallback(TlsContext *context, const uint8_t *pskIdentity,
   size_t pskIdentityLen)
{
   error_t error;

   //Debug message
   TRACE_INFO("TLS Server: PSK callback\r\n");

   //Check PSK identity
   if(pskIdentityLen == strlen(APP_CLIENT1_PSK_IDENTITY) &&
      !memcmp(pskIdentity, APP_CLIENT1_PSK_IDENTITY, pskIdentityLen))
   {
      //Set the pre-shared key to be used
      error = tlsSetPsk(context, client1Psk, sizeof(client1Psk));
   }
   else if(pskIdentityLen == strlen(APP_CLIENT2_PSK_IDENTITY) &&
      !memcmp(pskIdentity, APP_CLIENT2_PSK_IDENTITY, pskIdentityLen))
   {
      //Set the pre-shared key to be used
      error = tlsSetPsk(context, client2Psk, sizeof(client2Psk));
   }
   else
   {
      //Unknown PSK identity
      error = ERROR_UNKNOWN_IDENTITY;
   }

   //Return status code
   return error;
}


/**
 * @brief Display the contents of an array
 * @param[out] buffer Output buffer where to format the resulting string
 * @param[in] data Pointer to the data array
 * @param[in] length Number of bytes in the array
 * @return Length of the resulting string
 **/

size_t dumpArray(char_t *buffer, const uint8_t *data, size_t length)
{
   size_t i;
   size_t n;

   //Variable initialization
   n = 0;

   //Properly terminate the string
   buffer[0] = '\0';

   //Process input data
   for(i = 0; i < length; i++)
   {
      //Beginning of a new line?
      if(i != 0 && (i % 16) == 0)
      {
         n += sprintf(buffer + n, "\r\n        ");
      }

      //Display current data byte
      n += sprintf(buffer + n, "%02" PRIX8 " ", data[i]);
   }

   //Return the length of the resulting string
   return n;
}
