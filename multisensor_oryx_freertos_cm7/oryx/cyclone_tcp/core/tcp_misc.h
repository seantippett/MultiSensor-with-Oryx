/**
 * @file tcp_misc.h
 * @brief Helper functions for TCP
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

#ifndef _TCP_MISC_H
#define _TCP_MISC_H

//Dependencies
#include "core/tcp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TCP related functions
error_t tcpSendSegment(Socket *socket, uint8_t flags, uint32_t seqNum,
   uint32_t ackNum, size_t length, bool_t addToQueue);

error_t tcpSendResetSegment(Socket *socket, uint32_t seqNum);

error_t tcpRejectSegment(NetInterface *interface,
   const IpPseudoHeader *pseudoHeader, const TcpHeader *segment, size_t length);

error_t tcpAddOption(TcpHeader *segment, uint8_t kind, const void *value,
   uint8_t length);

const TcpOption *tcpGetOption(const TcpHeader *segment, uint8_t kind);

uint32_t tcpGenerateInitialSeqNum(const IpAddr *localIpAddr,
   uint16_t localPort, const IpAddr *remoteIpAddr, uint16_t remotePort);

error_t tcpCheckSeqNum(Socket *socket, const TcpHeader *segment, size_t length);
error_t tcpCheckSyn(Socket *socket, const TcpHeader *segment, size_t length);
error_t tcpCheckAck(Socket *socket, const TcpHeader *segment, size_t length);

bool_t tcpIsDuplicateSyn(Socket *socket, const IpPseudoHeader *pseudoHeader,
   const TcpHeader *segment);

bool_t tcpIsDuplicateAck(Socket *socket, const TcpHeader *segment,
   size_t length);

void tcpFastRetransmit(Socket *socket);
void tcpFastRecovery(Socket *socket, const TcpHeader *segment, uint_t n);
void tcpFastLossRecovery(Socket *socket, const TcpHeader *segment);

void tcpProcessSegmentData(Socket *socket, const TcpHeader *segment,
   const NetBuffer *buffer, size_t offset, size_t length);

void tcpDeleteControlBlock(Socket *socket);

void tcpUpdateRetransmitQueue(Socket *socket);
void tcpFlushRetransmitQueue(Socket *socket);

void tcpFlushSynQueue(Socket *socket);

void tcpUpdateSackBlocks(Socket *socket, uint32_t *leftEdge, uint32_t *rightEdge);
void tcpUpdateSendWindow(Socket *socket, const TcpHeader *segment);
void tcpUpdateReceiveWindow(Socket *socket);

bool_t tcpComputeRto(Socket *socket);
error_t tcpRetransmitSegment(Socket *socket);
error_t tcpNagleAlgo(Socket *socket, uint_t flags);

void tcpChangeState(Socket *socket, TcpState newState);

void tcpUpdateEvents(Socket *socket);
uint_t tcpWaitForEvents(Socket *socket, uint_t eventMask, systime_t timeout);

void tcpWriteTxBuffer(Socket *socket, uint32_t seqNum,
   const uint8_t *data, size_t length);

error_t tcpReadTxBuffer(Socket *socket, uint32_t seqNum,
   NetBuffer *buffer, size_t length);

void tcpWriteRxBuffer(Socket *socket, uint32_t seqNum,
   const NetBuffer *data, size_t dataOffset, size_t length);

void tcpReadRxBuffer(Socket *socket, uint32_t seqNum, uint8_t *data,
   size_t length);

void tcpDumpHeader(const TcpHeader *segment, size_t length, uint32_t iss,
   uint32_t irs);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
