/**
 * @file tls13_ticket.h
 * @brief TLS 1.3 session tickets
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSSL Eval.
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

#ifndef _TLS13_TICKET_H
#define _TLS13_TICKET_H

//Dependencies
#include "tls.h"

//TLS related functions
bool_t tls13IsTicketValid(TlsContext *context);

error_t tls13SaveSessionTicket(const TlsContext *context,
   TlsSessionState *session);

error_t tls13RestoreSessionTicket(TlsContext *context,
   const TlsSessionState *session);

error_t tls13GenerateTicket(TlsContext *context,
   const Tls13NewSessionTicket *message, uint8_t *ticket, size_t *length);

error_t tls13VerifyTicket(TlsContext *context, const uint8_t *ticket,
   size_t length, uint32_t obfuscatedTicketAge);

#endif
