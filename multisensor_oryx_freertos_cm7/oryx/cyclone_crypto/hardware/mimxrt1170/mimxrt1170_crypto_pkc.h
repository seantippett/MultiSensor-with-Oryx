/**
 * @file mimxrt1170_crypto_pkc.h
 * @brief i.MX RT1170 public-key hardware accelerator
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneCRYPTO Eval.
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

#ifndef _MIMXRT1170_CRYPTO_PKC_H
#define _MIMXRT1170_CRYPTO_PKC_H

//Dependencies
#include "core/crypto.h"

//Public-key hardware accelerator
#ifndef MIMXRT1170_CRYPTO_PKC_SUPPORT
   #define MIMXRT1170_CRYPTO_PKC_SUPPORT DISABLED
#elif (MIMXRT1170_CRYPTO_PKC_SUPPORT != ENABLED && MIMXRT1170_CRYPTO_PKC_SUPPORT != DISABLED)
   #error MIMXRT1170_CRYPTO_PKC_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief PKHA primitive arguments
 **/

typedef struct
{
   uint8_t a[512];
   uint8_t b[512];
   uint8_t e[512];
   uint8_t p[512];
   uint8_t r[512];
} PkhaArgs;


/**
 * @brief PKHA ECC primitive arguments
 **/

typedef struct
{
   uint8_t p[66];
   uint8_t a[66];
   uint8_t b[66];
   uint8_t d[66];
   uint8_t gx[66];
   uint8_t gy[66];
   uint8_t qx[66];
   uint8_t qy[66];
} PkhaEccArgs;


//C++ guard
#ifdef __cplusplus
}
#endif

#endif
