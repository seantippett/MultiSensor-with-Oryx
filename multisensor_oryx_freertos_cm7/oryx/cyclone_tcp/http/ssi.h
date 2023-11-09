/**
 * @file ssi.h
 * @brief SSI (Server Side Includes)
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

#ifndef _SSI_H
#define _SSI_H

//Dependencies
#include "http/http_server.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//SSI related functions
error_t ssiExecuteScript(HttpConnection *connection, const char_t *uri, uint_t level);

error_t ssiProcessCommand(HttpConnection *connection,
   const char_t *tag, size_t length, const char_t *uri, uint_t level);

error_t ssiProcessIncludeCommand(HttpConnection *connection,
   const char_t *tag, size_t length, const char_t *uri, uint_t level);

error_t ssiProcessEchoCommand(HttpConnection *connection, const char_t *tag,
   size_t length);

error_t ssiProcessExecCommand(HttpConnection *connection, const char_t *tag,
   size_t length);

error_t ssiSearchTag(const char_t *s, size_t sLen, const char_t *tag,
   size_t tagLen, size_t *pos);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
