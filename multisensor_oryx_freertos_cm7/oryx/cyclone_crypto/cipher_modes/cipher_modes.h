/**
 * @file cipher_modes.h
 * @brief Block cipher modes of operation
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

#ifndef _CIPHER_MODES_H
#define _CIPHER_MODES_H

//Dependencies
#include "core/crypto.h"

//ECB mode support?
#if (ECB_SUPPORT == ENABLED)
   #include "cipher_modes/ecb.h"
#endif

//CBC mode support?
#if (CBC_SUPPORT == ENABLED)
   #include "cipher_modes/cbc.h"
#endif

//CFB mode support?
#if (CFB_SUPPORT == ENABLED)
   #include "cipher_modes/cfb.h"
#endif

//OFB mode support?
#if (OFB_SUPPORT == ENABLED)
   #include "cipher_modes/ofb.h"
#endif

//CTR mode support?
#if (CTR_SUPPORT == ENABLED)
   #include "cipher_modes/ctr.h"
#endif

//XTS mode support?
#if (XTS_SUPPORT == ENABLED)
   #include "cipher_modes/xts.h"
#endif

#endif
