/**
 * @file eddsa.c
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

//Switch to the appropriate trace level
#define TRACE_LEVEL CRYPTO_TRACE_LEVEL

//Dependencies
#include "core/crypto.h"
#include "ecc/eddsa.h"
#include "mpi/mpi.h"
#include "debug.h"

//Check crypto library configuration
#if (ED25519_SUPPORT == ENABLED || ED448_SUPPORT == ENABLED)


/**
 * @brief Initialize an EdDSA public key
 * @param[in] key Pointer to the EdDSA public key to initialize
 **/

void eddsaInitPublicKey(EddsaPublicKey *key)
{
   //Initialize multiple precision integer
   mpiInit(&key->q);
}


/**
 * @brief Release an EdDSA public key
 * @param[in] key Pointer to the EdDSA public key to free
 **/

void eddsaFreePublicKey(EddsaPublicKey *key)
{
   //Free multiple precision integer
   mpiFree(&key->q);
}


/**
 * @brief Initialize an EdDSA private key
 * @param[in] key Pointer to the EdDSA private key to initialize
 **/

void eddsaInitPrivateKey(EddsaPrivateKey *key)
{
   //Initialize multiple precision integers
   mpiInit(&key->d);
   mpiInit(&key->q);

   //Initialize private key slot
   key->slot = -1;
}


/**
 * @brief Release an EdDSA private key
 * @param[in] key Pointer to the EdDSA public key to free
 **/

void eddsaFreePrivateKey(EddsaPrivateKey *key)
{
   //Free multiple precision integers
   mpiFree(&key->d);
   mpiFree(&key->q);
}


/**
 * @brief EdDSA key pair generation
 * @param[in] prngAlgo PRNG algorithm
 * @param[in] prngContext Pointer to the PRNG context
 * @param[in] curveInfo Elliptic curve parameters
 * @param[out] privateKey EdDSA private key
 * @param[out] publicKey EdDSA public key
 * @return Error code
 **/

error_t eddsaGenerateKeyPair(const PrngAlgo *prngAlgo, void *prngContext,
   const EcCurveInfo *curveInfo, EddsaPrivateKey *privateKey,
   EddsaPublicKey *publicKey)
{
   error_t error;

   //Generate a private key
   error = eddsaGeneratePrivateKey(prngAlgo, prngContext, curveInfo,
      privateKey);

   //Check status code
   if(!error)
   {
      //Derive the public key from the private key
      error = eddsaGeneratePublicKey(curveInfo, privateKey, publicKey);
   }

   //Return status code
   return error;
}


/**
 * @brief EdDSA private key generation
 * @param[in] prngAlgo PRNG algorithm
 * @param[in] prngContext Pointer to the PRNG context
 * @param[in] curveInfo Elliptic curve parameters
 * @param[out] privateKey EdDSA private key
 * @return Error code
 **/

error_t eddsaGeneratePrivateKey(const PrngAlgo *prngAlgo, void *prngContext,
   const EcCurveInfo *curveInfo, EddsaPrivateKey *privateKey)
{
   error_t error;

#if (ED25519_SUPPORT == ENABLED)
   //Ed25519 algorithm?
   if(curveInfo == ED25519_CURVE)
   {
      uint8_t rawPrivateKey[ED25519_PRIVATE_KEY_LEN];

      //Generate an Ed25519 private key
      error = ed25519GeneratePrivateKey(prngAlgo, prngContext, rawPrivateKey);

      //Check status code
      if(!error)
      {
         //Import the Ed25519 private key
         error = mpiImport(&privateKey->d, rawPrivateKey, ED25519_PRIVATE_KEY_LEN,
            MPI_FORMAT_LITTLE_ENDIAN);
      }
   }
   else
#endif
#if (ED448_SUPPORT == ENABLED)
   //Ed448 algorithm?
   if(curveInfo == ED448_CURVE)
   {
      uint8_t rawPrivateKey[ED448_PRIVATE_KEY_LEN];

      //Generate an Ed448 private key
      error = ed448GeneratePrivateKey(prngAlgo, prngContext, rawPrivateKey);

      //Check status code
      if(!error)
      {
         //Import the Ed448 private key
         error = mpiImport(&privateKey->d, rawPrivateKey, ED448_PRIVATE_KEY_LEN,
            MPI_FORMAT_LITTLE_ENDIAN);
      }
   }
   else
#endif
   //Unknown algorithm?
   {
      //Report an error
      error = ERROR_INVALID_PARAMETER;
   }

   //Return status code
   return error;
}


/**
 * @brief Derive the public key from an EdDSA private key
 * @param[in] curveInfo Elliptic curve parameters
 * @param[in] privateKey EdDSA private key
 * @param[out] publicKey EdDSA public key
 * @return Error code
 **/

error_t eddsaGeneratePublicKey(const EcCurveInfo *curveInfo,
   const EddsaPrivateKey *privateKey, EddsaPublicKey *publicKey)
{
   error_t error;

#if (ED25519_SUPPORT == ENABLED)
   //Ed25519 algorithm?
   if(curveInfo == ED25519_CURVE)
   {
      uint8_t rawPrivateKey[ED25519_PRIVATE_KEY_LEN];
      uint8_t rawPublicKey[ED25519_PUBLIC_KEY_LEN];

      //Export the Ed25519 private key
      error = mpiExport(&privateKey->d, rawPrivateKey, ED25519_PRIVATE_KEY_LEN,
         MPI_FORMAT_LITTLE_ENDIAN);

      //Check status code
      if(!error)
      {
         //Derive the public key from the private key
         error = ed25519GeneratePublicKey(rawPrivateKey, rawPublicKey);
      }

      //Check status code
      if(!error)
      {
         //Import the Ed25519 public key
         error = mpiImport(&publicKey->q, rawPublicKey, ED25519_PUBLIC_KEY_LEN,
            MPI_FORMAT_LITTLE_ENDIAN);
      }
   }
   else
#endif
#if (ED448_SUPPORT == ENABLED)
   //Ed448 algorithm?
   if(curveInfo == ED448_CURVE)
   {
      uint8_t rawPrivateKey[ED448_PRIVATE_KEY_LEN];
      uint8_t rawPublicKey[ED448_PUBLIC_KEY_LEN];

      //Export the Ed448 private key
      error = mpiExport(&privateKey->d, rawPrivateKey, ED448_PRIVATE_KEY_LEN,
         MPI_FORMAT_LITTLE_ENDIAN);

      //Check status code
      if(!error)
      {
         //Derive the public key from the private key
         error = ed448GeneratePublicKey(rawPrivateKey, rawPublicKey);
      }

      //Check status code
      if(!error)
      {
         //Import the Ed448 public key
         error = mpiImport(&publicKey->q, rawPublicKey, ED448_PUBLIC_KEY_LEN,
            MPI_FORMAT_LITTLE_ENDIAN);
      }
   }
   else
#endif
   //Unknown algorithm?
   {
      //Report an error
      error = ERROR_INVALID_PARAMETER;
   }

   //Return status code
   return error;
}

#endif
