/**
 * @file net_legacy.h
 * @brief Legacy definitions
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

#ifndef _NET_LEGACY_H
#define _NET_LEGACY_H

//Check compiler
#if !defined(_WIN32)

//Deprecated properties
#ifdef TCP_SYN_QUEUE_SIZE
   #warning TCP_SYN_QUEUE_SIZE property is deprecated. TCP_DEFAULT_SYN_QUEUE_SIZE should be used instead.
   #define TCP_DEFAULT_SYN_QUEUE_SIZE TCP_SYN_QUEUE_SIZE
#endif

#ifdef MAC_FILTER_MAX_SIZE
   //#warning MAC_FILTER_MAX_SIZE property is deprecated. MAC_ADDR_FILTER_SIZE should be used instead.
   #define MAC_ADDR_FILTER_SIZE MAC_FILTER_MAX_SIZE
#endif

#ifdef IPV4_FILTER_MAX_SIZE
   //#warning IPV4_FILTER_MAX_SIZE property is deprecated. IPV4_MULTICAST_FILTER_SIZE should be used instead.
   #define IPV4_MULTICAST_FILTER_SIZE IPV4_FILTER_MAX_SIZE
#endif

#ifdef IPV6_FILTER_MAX_SIZE
   //#warning IPV6_FILTER_MAX_SIZE property is deprecated. IPV6_MULTICAST_FILTER_SIZE should be used instead.
   #define IPV6_MULTICAST_FILTER_SIZE IPV6_FILTER_MAX_SIZE
#endif

#ifdef IPV4_MAX_DNS_SERVERS
   //#warning IPV4_MAX_DNS_SERVERS property is deprecated. IPV4_DNS_SERVER_LIST_SIZE should be used instead.
   #define IPV4_DNS_SERVER_LIST_SIZE IPV4_MAX_DNS_SERVERS
#endif

#ifdef IPV6_MAX_DNS_SERVERS
   //#warning IPV6_MAX_DNS_SERVERS property is deprecated. IPV6_DNS_SERVER_LIST_SIZE should be used instead.
   #define IPV6_DNS_SERVER_LIST_SIZE IPV6_MAX_DNS_SERVERS
#endif

#ifdef NET_TICK_STACK_SIZE
   #warning NET_TICK_STACK_SIZE property is deprecated and should be removed from net_config.h. The TCP/IP stack now uses a single task
#endif

#ifdef NET_TICK_PRIORITY
   #warning NET_TICK_PRIORITY property is deprecated and should be removed from net_config.h. TCP/IP stack now uses a single task.
#endif

#ifdef NET_RX_STACK_SIZE
   #warning NET_RX_STACK_SIZE property is deprecated since the TCP/IP stack now uses a single task. NET_TASK_STACK_SIZE should be used instead.
   #define NET_TASK_STACK_SIZE NET_RX_STACK_SIZE
#endif

#ifdef NET_RX_PRIORITY
   #warning NET_RX_PRIORITY property is deprecated since the TCP/IP stack now uses a single task. NET_TASK_PRIORITY should be used instead.
   #define NET_TASK_PRIORITY NET_RX_PRIORITY
#endif

#endif

//Legacy definitions
#define SOCKET_TYPE_RAW SOCKET_TYPE_RAW_IP
#define SOCKET_PROTOCOL_ICMP SOCKET_IP_PROTO_ICMP
#define SOCKET_PROTOCOL_IGMP SOCKET_IP_PROTO_IGMP
#define SOCKET_PROTOCOL_TCP SOCKET_IP_PROTO_TCP
#define SOCKET_PROTOCOL_UDP SOCKET_IP_PROTO_UDP
#define SOCKET_PROTOCOL_ICMPV6 SOCKET_IP_PROTO_ICMPV6

#ifdef SMTP_DEFAULT_TIMEOUT
   #define SMTP_CLIENT_DEFAULT_TIMEOUT SMTP_DEFAULT_TIMEOUT
#endif

#ifdef SMTP_MAX_LINE_LENGTH
   #define SMTP_CLIENT_MAX_LINE_LENGTH SMTP_MAX_LINE_LENGTH
#endif

#ifdef SMTP_TLS_SUPPORT
   #define SMTP_CLIENT_TLS_SUPPORT SMTP_TLS_SUPPORT
#endif

#ifdef SMTP_LOGIN_AUTH_SUPPORT
   #define SMTP_CLIENT_LOGIN_AUTH_SUPPORT SMTP_LOGIN_AUTH_SUPPORT
#endif

#ifdef SMTP_PLAIN_AUTH_SUPPORT
   #define SMTP_CLIENT_PLAIN_AUTH_SUPPORT SMTP_PLAIN_AUTH_SUPPORT
#endif

#ifdef SMTP_CRAM_MD5_AUTH_SUPPORT
   #define SMTP_CLIENT_CRAM_MD5_AUTH_SUPPORT SMTP_CRAM_MD5_AUTH_SUPPORT
#endif

#define DhcpClientCtx DhcpClientContext
#define Dhcpv6ClientCtx Dhcpv6ClientContext

#ifdef SNMP_AGENT_MAX_MIB_COUNT
   #define SNMP_AGENT_MAX_MIBS SNMP_AGENT_MAX_MIB_COUNT
#endif

#ifdef SNMP_AGENT_MAX_USER_COUNT
   #define SNMP_AGENT_MAX_USERS SNMP_AGENT_MAX_USER_COUNT
#endif

#ifdef MAC_MULTICAST_FILTER_SIZE
   #define MAC_ADDR_FILTER_SIZE MAC_MULTICAST_FILTER_SIZE
#endif

#define ftpRegisterTlsInitCallback ftpClientRegisterTlsInitCallback
#define ftpLogin ftpClientLoginEx
#define ftpGetWorkingDir ftpClientGetWorkingDir
#define ftpChangeWorkingDir ftpClientChangeWorkingDir
#define ftpChangeToParentDir ftpClientChangeToParentDir
#define ftpOpenDir ftpClientOpenDir
#define ftpReadDir ftpClientReadDir
#define ftpCloseDir ftpClientCloseDir
#define ftpMakeDir ftpClientCreateDir
#define ftpRemoveDir ftpClientDeleteDir
#define ftpOpenFile ftpClientOpenFile
#define ftpWriteFile(context, data, length, flags) ftpClientWriteFile(context, data, length, NULL, flags)
#define ftpReadFile ftpClientReadFile
#define ftpCloseFile ftpClientCloseFile
#define ftpRenameFile ftpClientRenameFile
#define ftpDeleteFile ftpClientDeleteFile
#define ftpClose ftpClientDeinit

#define FTP_NO_SECURITY       FTP_MODE_PLAINTEXT
#define FTP_IMPLICIT_SECURITY FTP_MODE_IMPLICIT_TLS
#define FTP_EXPLICIT_SECURITY FTP_MODE_EXPLICIT_TLS
#define FTP_ACTIVE_MODE       FTP_MODE_ACTIVE
#define FTP_PASSIVE_MODE      FTP_MODE_PASSIVE

#define FTP_FOR_READING   FTP_FILE_MODE_READ
#define FTP_FOR_WRITING   FTP_FILE_MODE_WRITE
#define FTP_FOR_APPENDING FTP_FILE_MODE_APPEND
#define FTP_BINARY_TYPE   FTP_FILE_MODE_BINARY
#define FTP_TEXT_TYPE     FTP_FILE_MODE_TEXT

#ifdef FTP_CLIENT_SOCKET_MIN_TX_BUFFER_SIZE
   #define FTP_CLIENT_MIN_TCP_BUFFER_SIZE FTP_CLIENT_SOCKET_MIN_TX_BUFFER_SIZE
#endif

#ifdef FTP_CLIENT_SOCKET_MAX_TX_BUFFER_SIZE
   #define FTP_CLIENT_MAX_TCP_BUFFER_SIZE FTP_CLIENT_SOCKET_MAX_TX_BUFFER_SIZE
#endif

#ifdef FTP_SERVER_CTRL_SOCKET_BUFFER_SIZE
   #define FTP_SERVER_MIN_TCP_BUFFER_SIZE FTP_SERVER_CTRL_SOCKET_BUFFER_SIZE
#endif

#ifdef FTP_SERVER_DATA_SOCKET_BUFFER_SIZE
   #define FTP_SERVER_MAX_TCP_BUFFER_SIZE FTP_SERVER_DATA_SOCKET_BUFFER_SIZE
#endif

#define NET_VERSION_STRING CYCLONE_TCP_VERSION_STRING
#define NET_MAJOR_VERSION CYCLONE_TCP_MAJOR_VERSION
#define NET_MINOR_VERSION CYCLONE_TCP_MINOR_VERSION
#define NET_REV_NUMBER CYCLONE_TCP_REV_NUMBER

#define ftpClientMakeDir ftpClientCreateDir
#define ftpClientRemoveDir ftpClientDeleteDir

#define MQTT_PROTOCOL_LEVEL_3_1 MQTT_VERSION_3_1
#define MQTT_PROTOCOL_LEVEL_3_1_1 MQTT_VERSION_3_1_1
#define mqttClientSetProtocolLevel mqttClientSetVersion

#define socketBindToInterface socketSetInterface

#define netMutex (netContext.mutex)
#define netEvent (netContext.event)
#define netTaskRunning (netContext.running)
#define netTimestamp (netContext.timestamp)
#define netInterface (netContext.interfaces)

#ifdef IGMP_SUPPORT
   #define IGMP_HOST_SUPPORT IGMP_SUPPORT
#endif

#ifdef COAP_SERVER_STACK_SIZE
   #define COAP_SERVER_TASK_STACK_SIZE COAP_SERVER_STACK_SIZE
#endif

#ifdef COAP_SERVER_PRIORITY
   #define COAP_SERVER_TASK_PRIORITY COAP_SERVER_PRIORITY
#endif

#ifdef DHCPV6_RELAY_STACK_SIZE
   #define DHCPV6_RELAY_TASK_STACK_SIZE DHCPV6_RELAY_STACK_SIZE
#endif

#ifdef DHCPV6_RELAY_PRIORITY
   #define DHCPV6_RELAY_TASK_PRIORITY DHCPV6_RELAY_PRIORITY
#endif

#ifdef FTP_SERVER_STACK_SIZE
   #define FTP_SERVER_TASK_STACK_SIZE FTP_SERVER_STACK_SIZE
#endif

#ifdef FTP_SERVER_PRIORITY
   #define FTP_SERVER_TASK_PRIORITY FTP_SERVER_PRIORITY
#endif

#ifdef HTTP_SERVER_STACK_SIZE
   #define HTTP_SERVER_TASK_STACK_SIZE HTTP_SERVER_STACK_SIZE
#endif

#ifdef HTTP_SERVER_PRIORITY
   #define HTTP_SERVER_TASK_PRIORITY HTTP_SERVER_PRIORITY
#endif

#ifdef ICECAST_CLIENT_STACK_SIZE
   #define ICECAST_CLIENT_TASK_STACK_SIZE ICECAST_CLIENT_STACK_SIZE
#endif

#ifdef ICECAST_CLIENT_PRIORITY
   #define ICECAST_CLIENT_TASK_PRIORITY ICECAST_CLIENT_PRIORITY
#endif

#ifdef MODBUS_SERVER_STACK_SIZE
   #define MODBUS_SERVER_TASK_STACK_SIZE MODBUS_SERVER_STACK_SIZE
#endif

#ifdef MODBUS_SERVER_PRIORITY
   #define MODBUS_SERVER_TASK_PRIORITY MODBUS_SERVER_PRIORITY
#endif

#ifdef SNMP_AGENT_STACK_SIZE
   #define SNMP_AGENT_TASK_STACK_SIZE SNMP_AGENT_STACK_SIZE
#endif

#ifdef SNMP_AGENT_PRIORITY
   #define SNMP_AGENT_TASK_PRIORITY SNMP_AGENT_PRIORITY
#endif

#ifdef TFTP_SERVER_STACK_SIZE
   #define TFTP_SERVER_TASK_STACK_SIZE TFTP_SERVER_STACK_SIZE
#endif

#ifdef TFTP_SERVER_PRIORITY
   #define TFTP_SERVER_TASK_PRIORITY TFTP_SERVER_PRIORITY
#endif

#define icmpEnableEchoRequest icmpEnableEchoRequests
#define icmpEnableBroadcastEchoRequest icmpEnableBroadcastEchoRequests
#define icmpv6EnableEchoRequest icmpv6EnableEchoRequests
#define icmpv6EnableMulticastEchoRequest icmpv6EnableMulticastEchoRequests

#endif
