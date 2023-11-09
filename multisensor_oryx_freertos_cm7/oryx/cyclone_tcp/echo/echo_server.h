/**
 * @file echo_server.h
 * @brief Echo server
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

#ifndef _ECHO_SERVER_H
#define _ECHO_SERVER_H

//Dependencies
#include "core/net.h"
#include "core/socket.h"

//Echo server support
#ifndef ECHO_SERVER_SUPPORT
   #define ECHO_SERVER_SUPPORT DISABLED
#elif (ECHO_SERVER_SUPPORT != ENABLED && ECHO_SERVER_SUPPORT != DISABLED)
   #error ECHO_SERVER_SUPPORT parameter is not valid
#endif

//Stack size required to run the Echo server
#ifndef ECHO_SERVER_STACK_SIZE
   #define ECHO_SERVER_STACK_SIZE 500
#elif (ECHO_SERVER_STACK_SIZE < 1)
   #error ECHO_SERVER_STACK_SIZE parameter is not valid
#endif

//Priority at which the Echo server should run
#ifndef ECHO_SERVER_PRIORITY
   #define ECHO_SERVER_PRIORITY OS_TASK_PRIORITY_NORMAL
#endif

//TCP Echo service support
#ifndef ECHO_SERVER_TCP_SUPPORT
   #define ECHO_SERVER_TCP_SUPPORT ENABLED
#elif (ECHO_SERVER_TCP_SUPPORT != ENABLED && ECHO_SERVER_TCP_SUPPORT != DISABLED)
   #error ECHO_SERVER_TCP_SUPPORT parameter is not valid
#endif

//Maximum number of simultaneous TCP connections
#ifndef ECHO_SERVER_MAX_TCP_CONNECTIONS
   #define ECHO_SERVER_MAX_TCP_CONNECTIONS 2
#elif (ECHO_SERVER_MAX_TCP_CONNECTIONS < 1)
   #error ECHO_SERVER_MAX_TCP_CONNECTIONS parameter is not valid
#endif

//Size of the buffer for input/output operations (TCP)
#ifndef ECHO_SERVER_TCP_BUFFER_SIZE
   #define ECHO_SERVER_TCP_BUFFER_SIZE 512
#elif (ECHO_SERVER_TCP_BUFFER_SIZE < 1)
   #error ECHO_SERVER_TCP_BUFFER_SIZE parameter is not valid
#endif

//UDP Echo service support
#ifndef ECHO_SERVER_UDP_SUPPORT
   #define ECHO_SERVER_UDP_SUPPORT ENABLED
#elif (ECHO_SERVER_UDP_SUPPORT != ENABLED && ECHO_SERVER_UDP_SUPPORT != DISABLED)
   #error ECHO_SERVER_UDP_SUPPORT parameter is not valid
#endif

//Size of the buffer for input/output operations (UDP)
#ifndef ECHO_SERVER_UDP_BUFFER_SIZE
   #define ECHO_SERVER_UDP_BUFFER_SIZE 1472
#elif (ECHO_SERVER_UDP_BUFFER_SIZE < 1)
   #error ECHO_SERVER_UDP_BUFFER_SIZE parameter is not valid
#endif

//Idle connection timeout
#ifndef ECHO_SERVER_TIMEOUT
   #define ECHO_SERVER_TIMEOUT 30000
#elif (ECHO_SERVER_TIMEOUT < 1)
   #error ECHO_SERVER_TIMEOUT parameter is not valid
#endif

//Echo server tick interval
#ifndef ECHO_SERVER_TICK_INTERVAL
   #define ECHO_SERVER_TICK_INTERVAL 1000
#elif (ECHO_SERVER_TICK_INTERVAL < 100)
   #error ECHO_SERVER_TICK_INTERVAL parameter is not valid
#endif

//Application specific context
#ifndef ECHO_SERVER_PRIVATE_CONTEXT
   #define ECHO_SERVER_PRIVATE_CONTEXT
#endif

//Echo service port number
#define ECHO_PORT 7

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief TCP connection state
 **/

typedef enum
{
   ECHO_TCP_CONNECTION_STATE_CLOSED = 0,
   ECHO_TCP_CONNECTION_STATE_OPEN   = 1
} EchoTcpConnectionState;


/**
 * @brief Echo server settings
 **/

typedef struct
{
   NetInterface *interface; ///<Underlying network interface
   uint16_t port;           ///<Echo server port number
} EchoServerSettings;


/**
 * @brief Echo TCP connection
 **/

typedef struct
{
   EchoTcpConnectionState state;               ///<Connection state
   Socket *socket;                             ///<Underlying TCP socket
   systime_t timestamp;                        ///<Time stamp
   char_t buffer[ECHO_SERVER_TCP_BUFFER_SIZE]; ///<Memory buffer for input/output operations (TCP)
   size_t bufferLen;                           ///<Length of the buffer, in bytes
   size_t bufferPos;                           ///<Current position in the buffer
} EchoTcpConnection;


/**
 * @brief Echo server context
 **/

typedef struct
{
   EchoServerSettings settings;                   ///<User settings
   bool_t running;                                ///<Operational state of the Echo server
   bool_t stop;                                   ///<Stop request
   OsEvent event;                                 ///<Event object used to poll the sockets
   OsTaskId taskId;                               ///<Task identifier
#if (OS_STATIC_TASK_SUPPORT == ENABLED)
   OsTaskTcb taskTcb;                             ///<Task control block
   OsStackType taskStack[ECHO_SERVER_STACK_SIZE]; ///<Task stack
#endif
#if (ECHO_SERVER_TCP_SUPPORT == ENABLED)
   Socket *tcpSocket;                             ///<Listening TCP socket
   EchoTcpConnection tcpConnection[ECHO_SERVER_MAX_TCP_CONNECTIONS]; ///<TCP connections
#endif
#if (ECHO_SERVER_UDP_SUPPORT == ENABLED)
   Socket *udpSocket;                             ///<UDP socket
   char_t udpBuffer[ECHO_SERVER_UDP_BUFFER_SIZE]; ///<Memory buffer for input/output operations (UDP)
#endif
   ECHO_SERVER_PRIVATE_CONTEXT                    ///<Application specific context
} EchoServerContext;


//Echo server related functions
void echoServerGetDefaultSettings(EchoServerSettings *settings);

error_t echoServerInit(EchoServerContext *context,
   const EchoServerSettings *settings);

error_t echoServerStart(EchoServerContext *context);
error_t echoServerStop(EchoServerContext *context);

void echoServerTask(EchoServerContext *context);

void echoServerDeinit(EchoServerContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
