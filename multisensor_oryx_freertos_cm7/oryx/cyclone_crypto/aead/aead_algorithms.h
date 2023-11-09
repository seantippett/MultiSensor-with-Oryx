/**
 * @file cipher_algorithms.h
 * @brief Collection of AEAD algorithms
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

#ifndef _AEAD_ALGORITHMS_H
#define _AEAD_ALGORITHMS_H

//Dependencies
#include "core/crypto.h"

//GCM mode support?
#if (GCM_SUPPORT == ENABLED)
   #include "aead/gcm.h"
#endif

//CCM mode support?
#if (CCM_SUPPORT == ENABLED)
   #include "aead/ccm.h"
#endif

//ChaCha20Poly1305 support?
#if (CHACHA20_POLY1305_SUPPORT == ENABLED)
   #include "aead/chacha20_poly1305.h"
#endif

#endif
