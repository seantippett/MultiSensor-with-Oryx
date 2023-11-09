/**
 * @file stp_procedures.h
 * @brief Elements of procedures
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSTP Eval.
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

#ifndef _STP_PROCEDURES_H
#define _STP_PROCEDURES_H

//Dependencies
#include "stp/stp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//STP related functions
void stpInitProc(StpBridgeContext *context);
void stpTransmitConfigBpdu(StpBridgePort *port);
void stpRecordConfigInfo(StpBridgePort *port, const StpBpdu *bpdu);
void stpRecordConfigTimeoutValues(StpBridgeContext *context, const StpBpdu *bpdu);
void stpConfigBpduGeneration(StpBridgeContext *context);
void stpReplyToConfigBpdu(StpBridgePort *port);
void stpTransmitTcnBpdu(StpBridgeContext *context);
void stpConfigUpdate(StpBridgeContext *context);
void stpRootSelection(StpBridgeContext *context);
void stpDesignatedPortSelection(StpBridgeContext *context);
void stpBecomeDesignatedPort(StpBridgePort *port);
void stpPortStateSelection(StpBridgeContext *context);
void stpMakeForwarding(StpBridgePort *port);
void stpMakeBlocking(StpBridgePort *port);
void stpTopologyChangeDetection(StpBridgeContext *context);
void stpTopologyChangeAcked(StpBridgeContext *context);
void stpAckTopologyChange(StpBridgePort *port);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
