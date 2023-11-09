/**
 * @file gcm.h
 * @brief Galois/Counter Mode (GCM)
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

#ifndef _GCM_H
#define _GCM_H

//Dependencies
#include "core/crypto.h"

//Precalculated table width, in bits
#ifndef GCM_TABLE_W
   #define GCM_TABLE_W 4
#elif (GCM_TABLE_W != 4 && GCM_TABLE_W != 8)
   #error GCM_TABLE_W parameter is not valid
#endif

//4-bit or 8-bit precalculated table?
#if (GCM_TABLE_W == 4)
   #define GCM_TABLE_N 16
   #define GCM_REVERSE_BITS(n) reverseInt4(n)
#else
   #define GCM_TABLE_N 256
   #define GCM_REVERSE_BITS(n) reverseInt8(n)
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief GCM context
 **/

typedef struct
{
   const CipherAlgo *cipherAlgo; ///<Cipher algorithm
   void *cipherContext;          ///<Cipher algorithm context
   uint32_t m[GCM_TABLE_N][4];   ///<Precalculated table
} GcmContext;


//GCM related functions
error_t gcmInit(GcmContext *context, const CipherAlgo *cipherAlgo,
   void *cipherContext);

error_t gcmEncrypt(GcmContext *context, const uint8_t *iv,
   size_t ivLen, const uint8_t *a, size_t aLen, const uint8_t *p,
   uint8_t *c, size_t length, uint8_t *t, size_t tLen);

error_t gcmDecrypt(GcmContext *context, const uint8_t *iv,
   size_t ivLen, const uint8_t *a, size_t aLen, const uint8_t *c,
   uint8_t *p, size_t length, const uint8_t *t, size_t tLen);

void gcmMul(GcmContext *context, uint8_t *x);
void gcmXorBlock(uint8_t *x, const uint8_t *a, const uint8_t *b, size_t n);
void gcmIncCounter(uint8_t *ctr);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
