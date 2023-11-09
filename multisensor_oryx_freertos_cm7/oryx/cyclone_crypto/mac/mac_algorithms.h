/**
 * @file mac_algorithms.h
 * @brief Collection of MAC algorithms
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

#ifndef _MAC_ALGORITHMS_H
#define _MAC_ALGORITHMS_H

//Dependencies
#include "core/crypto.h"

//CMAC support?
#if (CMAC_SUPPORT == ENABLED)
   #include "mac/cmac.h"
#endif

//HMAC support?
#if (HMAC_SUPPORT == ENABLED)
   #include "mac/hmac.h"
#endif

//GMAC support?
#if (GMAC_SUPPORT == ENABLED)
   #include "mac/gmac.h"
#endif

//KMAC support?
#if (KMAC_SUPPORT == ENABLED)
   #include "mac/kmac.h"
#endif

//XCBC-MAC support?
#if (XCBC_MAC_SUPPORT == ENABLED)
   #include "mac/xcbc_mac.h"
#endif

//Poly1305 support?
#if (POLY1305_SUPPORT == ENABLED)
   #include "mac/poly1305.h"
#endif

#endif
