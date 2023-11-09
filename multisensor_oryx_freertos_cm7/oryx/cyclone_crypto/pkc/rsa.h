/**
 * @file rsa.h
 * @brief RSA public-key cryptography standard
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

#ifndef _RSA_H
#define _RSA_H

//Dependencies
#include "core/crypto.h"
#include "hash/hash_algorithms.h"
#include "mpi/mpi.h"

//Maximum acceptable size for RSA modulus
#ifndef RSA_MAX_MODULUS_SIZE
   #define RSA_MAX_MODULUS_SIZE 4096
#elif (RSA_MAX_MODULUS_SIZE < 0)
   #error RSA_MAX_MODULUS_SIZE parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief RSA public key
 **/

typedef struct
{
   Mpi n; ///<Modulus
   Mpi e; ///<Public exponent
} RsaPublicKey;


/**
 * @brief RSA private key
 **/

typedef struct
{
   Mpi n;      ///<Modulus
   Mpi e;      ///<Public exponent
   Mpi d;      ///<Private exponent
   Mpi p;      ///<First factor
   Mpi q;      ///<Second factor
   Mpi dp;     ///<First factor's CRT exponent
   Mpi dq;     ///<Second factor's CRT exponent
   Mpi qinv;   ///<CRT coefficient
   int_t slot; ///<Private key slot
} RsaPrivateKey;


//RSA related constants
extern const uint8_t PKCS1_OID[8];
extern const uint8_t RSA_ENCRYPTION_OID[9];
extern const uint8_t MD2_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t MD5_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA1_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA224_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA256_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA384_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA512_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA512_256_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t SHA512_224_WITH_RSA_ENCRYPTION_OID[9];
extern const uint8_t RSASSA_PKCS1_V1_5_WITH_SHA3_224_OID[9];
extern const uint8_t RSASSA_PKCS1_V1_5_WITH_SHA3_256_OID[9];
extern const uint8_t RSASSA_PKCS1_V1_5_WITH_SHA3_384_OID[9];
extern const uint8_t RSASSA_PKCS1_V1_5_WITH_SHA3_512_OID[9];
extern const uint8_t RSASSA_PSS_OID[9];
extern const uint8_t MGF1_OID[9];

//RSA related functions
void rsaInitPublicKey(RsaPublicKey *key);
void rsaFreePublicKey(RsaPublicKey *key);
void rsaInitPrivateKey(RsaPrivateKey *key);
void rsaFreePrivateKey(RsaPrivateKey *key);

error_t rsaesPkcs1v15Encrypt(const PrngAlgo *prngAlgo, void *prngContext,
   const RsaPublicKey *key, const uint8_t *message, size_t messageLen,
   uint8_t *ciphertext, size_t *ciphertextLen);

error_t rsaesPkcs1v15Decrypt(const RsaPrivateKey *key,
   const uint8_t *ciphertext, size_t ciphertextLen, uint8_t *message,
   size_t messageSize, size_t *messageLen);

error_t rsaesOaepEncrypt(const PrngAlgo *prngAlgo, void *prngContext,
   const RsaPublicKey *key, const HashAlgo *hash, const char_t *label,
   const uint8_t *message, size_t messageLen, uint8_t *ciphertext,
   size_t *ciphertextLen);

error_t rsaesOaepDecrypt(const RsaPrivateKey *key, const HashAlgo *hash,
   const char_t *label, const uint8_t *ciphertext, size_t ciphertextLen,
   uint8_t *message, size_t messageSize, size_t *messageLen);

error_t rsassaPkcs1v15Sign(const RsaPrivateKey *key, const HashAlgo *hash,
   const uint8_t *digest, uint8_t *signature, size_t *signatureLen);

error_t rsassaPkcs1v15Verify(const RsaPublicKey *key, const HashAlgo *hash,
   const uint8_t *digest, const uint8_t *signature, size_t signatureLen);

error_t rsassaPssSign(const PrngAlgo *prngAlgo, void *prngContext,
   const RsaPrivateKey *key, const HashAlgo *hash, size_t saltLen,
   const uint8_t *digest, uint8_t *signature, size_t *signatureLen);

error_t rsassaPssVerify(const RsaPublicKey *key, const HashAlgo *hash,
   size_t saltLen, const uint8_t *digest, const uint8_t *signature,
   size_t signatureLen);

error_t rsaep(const RsaPublicKey *key, const Mpi *m, Mpi *c);
error_t rsadp(const RsaPrivateKey *key, const Mpi *c, Mpi *m);

error_t rsasp1(const RsaPrivateKey *key, const Mpi *m, Mpi *s);
error_t rsavp1(const RsaPublicKey *key, const Mpi *s, Mpi *m);

error_t emePkcs1v15Encode(const PrngAlgo *prngAlgo, void *prngContext,
   const uint8_t *message, size_t messageLen, uint8_t *em, size_t k);

uint32_t emePkcs1v15Decode(uint8_t *em, size_t k, size_t *messageLen);

error_t emeOaepEncode(const PrngAlgo *prngAlgo, void *prngContext,
   const HashAlgo *hash, const char_t *label, const uint8_t *message,
   size_t messageLen, uint8_t *em, size_t k);

uint32_t emeOaepDecode(const HashAlgo *hash, const char_t *label, uint8_t *em,
   size_t k, size_t *messageLen);

error_t emsaPkcs1v15Encode(const HashAlgo *hash,
   const uint8_t *digest, uint8_t *em, size_t emLen);

error_t emsaPkcs1v15Verify(const HashAlgo *hash, const uint8_t *digest,
   const uint8_t *em, size_t emLen);

error_t emsaPssEncode(const PrngAlgo *prngAlgo, void *prngContext,
   const HashAlgo *hash, size_t saltLen, const uint8_t *digest,
   uint8_t *em, uint_t emBits);

error_t emsaPssVerify(const HashAlgo *hash, size_t saltLen,
   const uint8_t *digest, uint8_t *em, uint_t emBits);

void mgf1(const HashAlgo *hash, HashContext *hashContext, const uint8_t *seed,
   size_t seedLen, uint8_t *data, size_t dataLen);

error_t rsaGenerateKeyPair(const PrngAlgo *prngAlgo, void *prngContext,
   size_t k, uint_t e, RsaPrivateKey *privateKey, RsaPublicKey *publicKey);

error_t rsaGeneratePrivateKey(const PrngAlgo *prngAlgo, void *prngContext,
   size_t k, uint_t e, RsaPrivateKey *privateKey);

error_t rsaGeneratePublicKey(const RsaPrivateKey *privateKey,
   RsaPublicKey *publicKey);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
