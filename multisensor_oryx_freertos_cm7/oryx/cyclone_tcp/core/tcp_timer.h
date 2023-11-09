/**
 * @file tcp_timer.h
 * @brief TCP timer management
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

#ifndef _TCP_TIMER_H
#define _TCP_TIMER_H

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TCP timer related functions
void tcpTick(void);

void tcpCheckRetransmitTimer(Socket *socket);
void tcpCheckPersistTimer(Socket *socket);
void tcpCheckKeepAliveTimer(Socket *socket);
void tcpCheckOverrideTimer(Socket *socket);
void tcpCheckFinWait2Timer(Socket *socket);
void tcpCheckTimeWaitTimer(Socket *socket);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
