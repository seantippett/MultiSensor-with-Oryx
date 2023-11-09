/**
 ******************************************************************************
 * @file    lwipopts.h
 * This file is based on \src\include\lwip\opt.h
 ******************************************************************************
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__


#if USE_RTOS

/**
 * SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection for certain
 * critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT 1

/**
 * NO_SYS==0: Use RTOS
 */
#define NO_SYS 0
/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN 1
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET 1

/**
 * LWIP_SO_RCVTIMEO==1: Enable receive timeout for sockets/netconns and
 * SO_RCVTIMEO processing.
 */
#define LWIP_SO_RCVTIMEO 1

#else
/**
 * NO_SYS==1: Bare metal lwIP
 */
#define NO_SYS       1
/**
 * LWIP_NETCONN==0: Disable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN 0
/**
 * LWIP_SOCKET==0: Disable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET  0

#endif


#define LWIP_NETCONN 1
#define LWIP_TCPIP_TIMEOUT 1
#define LWIP_NETCONN_SEM_PER_THREAD 0
#define LWIP_NETCONN_FULLDUPLEX 	0
#define LWIP_SOCKET	1
#define LWIP_COMPAT_SOCKETS	1
#define LWIP_POSIX_SOCKETS_IO_NAMES	1
#define LWIP_SOCKET_OFFSET	0
#define LWIP_TCP_KEEPALIVE	1
#define LWIP_SO_SNDTIMEO	1
#define LWIP_SO_RCVTIMEO	1
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 0
#define LWIP_SO_RCVBUF	1
#define LWIP_SO_LINGER	1
#define RECV_BUFSIZE_DEFAULT	2000000000
#define LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT	1000
#define SO_REUSE 1									// T.S. added so that we can re-use ports.  fixes crappy closures on IP ports.
#define SO_REUSE_RXTOALL 0
#define LWIP_FIONREAD_LINUXMODE 0
#define LWIP_SOCKET_SELECT	1
#define LWIP_SOCKET_POLL	1







/* ---------- Core locking ---------- */

#define LWIP_TCPIP_CORE_LOCKING 1

void sys_lock_tcpip_core(void);
#define LOCK_TCPIP_CORE() sys_lock_tcpip_core()

void sys_unlock_tcpip_core(void);
#define UNLOCK_TCPIP_CORE() sys_unlock_tcpip_core()

void sys_check_core_locking(void);
#define LWIP_ASSERT_CORE_LOCKED() sys_check_core_locking()

void sys_mark_tcpip_thread(void);
#define LWIP_MARK_TCPIP_THREAD() sys_mark_tcpip_thread()

/* ---------- Memory options ---------- */
/**
 * MEM_ALIGNMENT: should be set to the alignment of the CPU
 *    4 byte alignment -> #define MEM_ALIGNMENT 4
 *    2 byte alignment -> #define MEM_ALIGNMENT 2
 */
#ifndef MEM_ALIGNMENT
#define MEM_ALIGNMENT 4
#endif

/**
 * MEM_SIZE: the size of the heap memory. If the application will send
 * a lot of data that needs to be copied, this should be set high.
 */
#ifndef MEM_SIZE
#define MEM_SIZE (48 * 1024) //changed to 64x1024 on 2023 02 13.  T.S.   (36 * 1024) // was 22
#endif

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#ifndef MEMP_NUM_PBUF
#define MEMP_NUM_PBUF 25 // was 15
#endif
/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
   per active UDP "connection". */
#ifndef MEMP_NUM_UDP_PCB
#define MEMP_NUM_UDP_PCB 6
#endif
/* MEMP_NUM_TCP_PCB: the number of simulatenously active TCP
   connections. */
#ifndef MEMP_NUM_TCP_PCB
#define MEMP_NUM_TCP_PCB 10
#endif
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
#ifndef MEMP_NUM_TCP_PCB_LISTEN
#define MEMP_NUM_TCP_PCB_LISTEN 6
#endif
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
#ifndef MEMP_NUM_TCP_SEG
//#define MEMP_NUM_TCP_SEG 48 //66 // was 22
#define MEMP_NUM_TCP_SEG 6000 //T.S. changed to 6000 on 2023 02 12 because it needs to be at least as big as TCP_SND_QUEUELEN.
#endif
/* MEMP_NUM_SYS_TIMEOUT: the number of simulateously active
   timeouts. */
#ifndef MEMP_NUM_SYS_TIMEOUT
#define MEMP_NUM_SYS_TIMEOUT 10
#endif

/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#ifndef PBUF_POOL_SIZE
#define PBUF_POOL_SIZE 18 // was 9
#endif

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */
/* Default value is defined in lwip\src\include\lwip\opt.h as
 * LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_ENCAPSULATION_HLEN+PBUF_LINK_HLEN)*/

/* ---------- TCP options ---------- */
#ifndef LWIP_TCP
#define LWIP_TCP 1
#endif

#ifndef TCP_TTL
#define TCP_TTL 255
#endif

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */


 /*Strictly, queueing out-of-sequence packets is only necessary when packet loss is
  * expected, since it prevents resending all packets (e.g. packets 2, 3, 4) when
  * only one packet is lost (e.g. packet 2 is lost but 3 and 4 have been received
  * correctly: with TCP_QUEUE_OOSEQ disabled, packets 3 and 4 would be discarded
  * as they are out-of-sequence and would have to be resent in-sequence by the
  * remote host once packet 2 got through). However, even in environments where
  * packet loss isn't expected, it might still happen, so enabling this is recommended. */

/* T.S.  Turned this on 2023 02 12... this sounds like a great idea! */

#ifndef TCP_QUEUE_OOSEQ
#define TCP_QUEUE_OOSEQ 1
#endif

/* TCP Maximum segment size. */
/* T.S. This is 1460 for standard ethernet. */

#ifndef TCP_MSS
#define TCP_MSS (1500 - 40) /* TCP_MSS = (Ethernet MTU - IP header size - TCP header size) */
#endif

/* TCP sender buffer space (bytes). */

/*This limits the sender buffer space (in bytes): tcp_write only allows a limited amount of
 * bytes to be buffered (until acknowledged). For maximum throughput, set this to the same
 * value as TCP_WND (effectively disabling the extra-check). ATTENTION: keep in mind that
 * every active connection might buffer this amount of data, so make sure you have enough RAM
 * or limit the number of concurrently active connections!  */

#ifndef TCP_SND_BUF
//#define TCP_SND_BUF (12 * TCP_MSS) // 2    //ST this was 6 * TCP_MSS
#define TCP_SND_BUF (0xFFFF) // T.S.  It'd be nice to hold all of an Audio msg (i.e. 1 920 000 bytes).
#endif

/* TCP sender buffer space (pbufs). This must be at least = 2 *
   TCP_SND_BUF/TCP_MSS for things to work. */
#ifndef TCP_SND_QUEUELEN
#define TCP_SND_QUEUELEN (4 * TCP_SND_BUF) / TCP_MSS // 3
#endif

/* TCP receive window. */

/*The TCP window size can be adjusted by changing the define TCP_WND. However, do keep in mind that this
 * should be at least twice the size of TCP_MSS (thus on ethernet, where TCP_MSS is 1460, it should be
 * set to at least 2920). If memory allows it, set this as high as possible (16-bit, so 0xFFFF is the
 * highest value), but keep in mind that for every active connection, the full window may have to be buffered
 * until it is acknowledged by the remote side (although this buffer size can still be controlled
 * by TCP_SND_BUF and TCP_SND_QUEUELEN). The reason for "twice" are both the nagle algorithm and delayed
 * ACK from the remote peer.*/
#ifndef TCP_WND
//#define TCP_WND (4 * TCP_MSS)	//TS 2022 08 18
#define TCP_WND (0xFFFF)	//TS 2023 02 12
#endif

/* Enable backlog*/
#ifndef TCP_LISTEN_BACKLOG
#define TCP_LISTEN_BACKLOG 1
#endif

/* ---------- Network Interfaces options ---------- */
/* Support netif api (in netifapi.c). */
#ifndef LWIP_NETIF_API
#define LWIP_NETIF_API 1
#endif

/* ---------- ICMP options ---------- */
#ifndef LWIP_ICMP
#define LWIP_ICMP 1
#endif

/* ---------- RAW options ---------- */
#if !defined LWIP_RAW
#define LWIP_RAW 1
#endif

/* ---------- DHCP options ---------- */
#ifndef LWIP_DHCP
#define LWIP_DHCP 0
#endif

/* ---------- UDP options ---------- */
#ifndef LWIP_UDP
#define LWIP_UDP 1
#endif
#ifndef UDP_TTL
#define UDP_TTL 255
#endif

/* ---------- Statistics options ---------- */
#ifndef LWIP_STATS
#define LWIP_STATS 0
#endif
#ifndef LWIP_PROVIDE_ERRNO
#define LWIP_PROVIDE_ERRNO 1
#endif

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/

/*
Some MCU allow computing and verifying the IP, UDP, TCP and ICMP checksums by hardware:
 - To use this feature let the following define uncommented.
 - To disable it and process by CPU comment the  the checksum.
*/
//#define CHECKSUM_BY_HARDWARE

#ifdef CHECKSUM_BY_HARDWARE
/* CHECKSUM_GEN_IP==0: Generate checksums by hardware for outgoing IP packets.*/
#define CHECKSUM_GEN_IP 0
/* CHECKSUM_GEN_UDP==0: Generate checksums by hardware for outgoing UDP packets.*/
#define CHECKSUM_GEN_UDP 0
/* CHECKSUM_GEN_TCP==0: Generate checksums by hardware for outgoing TCP packets.*/
#define CHECKSUM_GEN_TCP 0
/* CHECKSUM_CHECK_IP==0: Check checksums by hardware for incoming IP packets.*/
#define CHECKSUM_CHECK_IP 0
/* CHECKSUM_CHECK_UDP==0: Check checksums by hardware for incoming UDP packets.*/
#define CHECKSUM_CHECK_UDP 0
/* CHECKSUM_CHECK_TCP==0: Check checksums by hardware for incoming TCP packets.*/
#define CHECKSUM_CHECK_TCP 0
#else
/* CHECKSUM_GEN_IP==1: Generate checksums in software for outgoing IP packets.*/
#define CHECKSUM_GEN_IP    1
/* CHECKSUM_GEN_UDP==1: Generate checksums in software for outgoing UDP packets.*/
#define CHECKSUM_GEN_UDP   1
/* CHECKSUM_GEN_TCP==1: Generate checksums in software for outgoing TCP packets.*/
#define CHECKSUM_GEN_TCP   1
/* CHECKSUM_CHECK_IP==1: Check checksums in software for incoming IP packets.*/
#define CHECKSUM_CHECK_IP  1
/* CHECKSUM_CHECK_UDP==1: Check checksums in software for incoming UDP packets.*/
#define CHECKSUM_CHECK_UDP 1
/* CHECKSUM_CHECK_TCP==1: Check checksums in software for incoming TCP packets.*/
#define CHECKSUM_CHECK_TCP 1
#endif

/**
 * DEFAULT_THREAD_STACKSIZE: The stack size used by any other lwIP thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#ifndef DEFAULT_THREAD_STACKSIZE
#define DEFAULT_THREAD_STACKSIZE 3000 // was 3000 but for whatever reason that exceeded the mem size (turned into 12000 somewhere)
#endif

/**
 * DEFAULT_THREAD_PRIO: The priority assigned to any other lwIP thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#ifndef DEFAULT_THREAD_PRIO
#define DEFAULT_THREAD_PRIO 3
#endif

/*
   ------------------------------------
   ---------- Debugging options ----------
   ------------------------------------
*/
#ifndef LWIP_DEBUG
#define LWIP_DEBUG
#endif

#ifdef LWIP_DEBUG
#define U8_F  "c"
#define S8_F  "c"
#define X8_F  "02x"
#define U16_F "u"
#define S16_F "d"
#define X16_F "x"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"
#define SZT_F "u"
#endif

#define TCPIP_MBOX_SIZE        32
#define TCPIP_THREAD_STACKSIZE 1024
#define TCPIP_THREAD_PRIO      10

/**
 * DEFAULT_RAW_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_RAW. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_RAW_RECVMBOX_SIZE 12

/**
 * DEFAULT_UDP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_UDP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_UDP_RECVMBOX_SIZE 12

/**
 * DEFAULT_TCP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_TCP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_TCP_RECVMBOX_SIZE 12

/**
 * DEFAULT_ACCEPTMBOX_SIZE: The mailbox size for the incoming connections.
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when the acceptmbox is created.
 */
#define DEFAULT_ACCEPTMBOX_SIZE 12

#if (LWIP_DNS || LWIP_IGMP || LWIP_IPV6) && !defined(LWIP_RAND)
/* When using IGMP or IPv6, LWIP_RAND() needs to be defined to a random-function returning an u32_t random value*/
#include "lwip/arch.h"
u32_t lwip_rand(void);
#define LWIP_RAND() lwip_rand()
#endif

#endif /* __LWIPOPTS_H__ */

/*****END OF FILE****/
