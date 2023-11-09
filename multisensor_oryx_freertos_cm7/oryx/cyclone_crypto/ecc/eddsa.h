/**
 * @file eddsa.h
 * @brief EdDSA (Edwards-Curve Digital Signature Algorithm)
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

#ifndef _EDDSA_H
#define _EDDSA_H

//Dependencies
#include "core/crypto.h"
#include "ecc/ec.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief EdDSA public key
 **/

typedef struct
{
   Mpi q; ///<Public key
} EddsaPublicKey;


/**
 * @brief EdDSA private key
 **/

typedef struct
{
   Mpi d;      ///<Private key
   Mpi q;      ///<Public key
   int_t slot; ///<Private key slot
} EddsaPrivateKey;


/**
 * @brief Message chunk descriptor
 **/

typedef struct
{
   const void *buffer;
   size_t length;
} EddsaMessageChunk;


//EdDSA related functions
void eddsaInitPublicKey(EddsaPublicKey *key);
void eddsaFreePublicKey(EddsaPublicKey *key);

void eddsaInitPrivateKey(EddsaPrivateKey *key);
void eddsaFreePrivateKey(EddsaPrivateKey *key);

error_t eddsaGenerateKeyPair(const PrngAlgo *prngAlgo, void *prngContext,
   const EcCurveInfo *curveInfo, EddsaPrivateKey *privateKey,
   EddsaPublicKey *publicKey);

error_t eddsaGeneratePrivateKey(const PrngAlgo *prngAlgo, void *prngContext,
   const EcCurveInfo *curveInfo, EddsaPrivateKey *privateKey);

error_t eddsaGeneratePublicKey(const EcCurveInfo *curveInfo,
   const EddsaPrivateKey *privateKey, EddsaPublicKey *publicKey);

//C++ guard
#ifdef __cplusplus
}
#endif

//Ed25519 supported?
#if (ED25519_SUPPORT == ENABLED)
   #include "ecc/ed25519.h"
#endif

//Ed448 supported?
#if (ED448_SUPPORT == ENABLED)
   #include "ecc/ed448.h"
#endif

#endif
