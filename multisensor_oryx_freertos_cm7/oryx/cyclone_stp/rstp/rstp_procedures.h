/**
 * @file rstp_procedures.h
 * @brief RSTP state machine procedures
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

#ifndef _RSTP_PROCEDURES_H
#define _RSTP_PROCEDURES_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//RSTP related functions
bool_t rstpBetterOrSameInfo(RstpBridgePort *port, RstpInfoIs newInfoIs);
void rstpClearReselectTree(RstpBridgeContext *context);
void rstpDisableForwarding(RstpBridgePort *port);
void rstpDisableLearning(RstpBridgePort *port);
void rstpEnableForwarding(RstpBridgePort *port);
void rstpEnableLearning(RstpBridgePort *port);
void rstpNewTcWhile(RstpBridgePort *port);
RstpRcvdInfo rstpRcvInfo(RstpBridgePort *port);
void rstpRecordAgreement(RstpBridgePort *port);
void rstpRecordDispute(RstpBridgePort *port);
void rstpRecordProposal(RstpBridgePort *port);
void rstpRecordPriority(RstpBridgePort *port);
void rstpRecordTimes(RstpBridgePort *port);
void rstpSetSyncTree(RstpBridgeContext *context);
void rstpSetReRootTree(RstpBridgeContext *context);
void rstpSetSelectedTree(RstpBridgeContext *context);
void rstpSetTcFlags(RstpBridgePort *port);
void rstpSetTcPropTree(RstpBridgePort *port);
void rstpTxConfig(RstpBridgePort *port);
void rstpTxRstp(RstpBridgePort *port);
void rstpTxTcn(RstpBridgePort *port);
void rstpUpdtBpduVersion(RstpBridgePort *port);
void rstpUpdtRcvdInfoWhile(RstpBridgePort *port);
void rstpUpdtRoleDisabledTree(RstpBridgeContext *context);
void rstpUpdtRolesTree(RstpBridgeContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
