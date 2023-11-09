/**
 * @file shake.h
 * @brief SHAKE128 and SHAKE256 extendable-output functions
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

#ifndef _SHAKE_H
#define _SHAKE_H

//Dependencies
#include "core/crypto.h"
#include "xof/keccak.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief SHAKE algorithm context
 **/

typedef struct
{
   KeccakContext keccakContext;
} ShakeContext;


//SHAKE related constants
extern const uint8_t shake128Oid[9];
extern const uint8_t shake256Oid[9];

//SHAKE related functions
error_t shakeCompute(uint_t strength, const void *input, size_t inputLen,
   uint8_t *output, size_t outputLen);

error_t shakeInit(ShakeContext *context, uint_t strength);
void shakeAbsorb(ShakeContext *context, const void *input, size_t length);
void shakeFinal(ShakeContext *context);
void shakeSqueeze(ShakeContext *context, uint8_t *output, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
