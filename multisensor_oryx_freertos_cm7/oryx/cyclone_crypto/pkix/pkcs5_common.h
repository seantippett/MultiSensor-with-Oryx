/**
 * @file pkcs5_common.h
 * @brief PKCS #5 common definitions
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

#ifndef _PKCS5_COMMON_H
#define _PKCS5_COMMON_H

//Dependencies
#include "core/crypto.h"

//RC2 encryption support (insecure)
#ifndef PKCS5_RC2_SUPPORT
   #define PKCS5_RC2_SUPPORT DISABLED
#elif (PKCS5_RC2_SUPPORT != ENABLED && PKCS5_RC2_SUPPORT != DISABLED)
   #error PKCS5_RC2_SUPPORT parameter is not valid
#endif

//DES encryption support (insecure)
#ifndef PKCS5_DES_SUPPORT
   #define PKCS5_DES_SUPPORT DISABLED
#elif (PKCS5_DES_SUPPORT != ENABLED && PKCS5_DES_SUPPORT != DISABLED)
   #error PKCS5_DES_SUPPORT parameter is not valid
#endif

//Triple DES encryption support (weak)
#ifndef PKCS5_3DES_SUPPORT
   #define PKCS5_3DES_SUPPORT DISABLED
#elif (PKCS5_3DES_SUPPORT != ENABLED && PKCS5_3DES_SUPPORT != DISABLED)
   #error PKCS5_3DES_SUPPORT parameter is not valid
#endif

//AES encryption support
#ifndef PKCS5_AES_SUPPORT
   #define PKCS5_AES_SUPPORT ENABLED
#elif (PKCS5_AES_SUPPORT != ENABLED && PKCS5_AES_SUPPORT != DISABLED)
   #error PKCS5_AES_SUPPORT parameter is not valid
#endif

//Camellia encryption support
#ifndef PKCS5_CAMELLIA_SUPPORT
   #define PKCS5_CAMELLIA_SUPPORT DISABLED
#elif (PKCS5_CAMELLIA_SUPPORT != ENABLED && PKCS5_CAMELLIA_SUPPORT != DISABLED)
   #error PKCS5_CAMELLIA_SUPPORT parameter is not valid
#endif

//ARIA encryption support
#ifndef PKCS5_ARIA_SUPPORT
   #define PKCS5_ARIA_SUPPORT DISABLED
#elif (PKCS5_ARIA_SUPPORT != ENABLED && PKCS5_ARIA_SUPPORT != DISABLED)
   #error PKCS5_ARIA_SUPPORT parameter is not valid
#endif

//SM4 encryption support
#ifndef PKCS5_SM4_SUPPORT
   #define PKCS5_SM4_SUPPORT DISABLED
#elif (PKCS5_SM4_SUPPORT != ENABLED && PKCS5_SM4_SUPPORT != DISABLED)
   #error PKCS5_SM4_SUPPORT parameter is not valid
#endif

//MD2 hash support (insecure)
#ifndef PKCS5_MD2_SUPPORT
   #define PKCS5_MD2_SUPPORT DISABLED
#elif (PKCS5_MD2_SUPPORT != ENABLED && PKCS5_MD2_SUPPORT != DISABLED)
   #error PKCS5_MD2_SUPPORT parameter is not valid
#endif

//MD5 hash support (insecure)
#ifndef PKCS5_MD5_SUPPORT
   #define PKCS5_MD5_SUPPORT DISABLED
#elif (PKCS5_MD5_SUPPORT != ENABLED && PKCS5_MD5_SUPPORT != DISABLED)
   #error PKCS5_MD5_SUPPORT parameter is not valid
#endif

//SHA-1 hash support (weak)
#ifndef PKCS5_SHA1_SUPPORT
   #define PKCS5_SHA1_SUPPORT DISABLED
#elif (PKCS5_SHA1_SUPPORT != ENABLED && PKCS5_SHA1_SUPPORT != DISABLED)
   #error PKCS5_SHA1_SUPPORT parameter is not valid
#endif

//SHA-224 hash support (weak)
#ifndef PKCS5_SHA224_SUPPORT
   #define PKCS5_SHA224_SUPPORT DISABLED
#elif (PKCS5_SHA224_SUPPORT != ENABLED && PKCS5_SHA224_SUPPORT != DISABLED)
   #error PKCS5_SHA224_SUPPORT parameter is not valid
#endif

//SHA-256 hash support
#ifndef PKCS5_SHA256_SUPPORT
   #define PKCS5_SHA256_SUPPORT ENABLED
#elif (PKCS5_SHA256_SUPPORT != ENABLED && PKCS5_SHA256_SUPPORT != DISABLED)
   #error PKCS5_SHA256_SUPPORT parameter is not valid
#endif

//SHA-384 hash support
#ifndef PKCS5_SHA384_SUPPORT
   #define PKCS5_SHA384_SUPPORT ENABLED
#elif (PKCS5_SHA384_SUPPORT != ENABLED && PKCS5_SHA384_SUPPORT != DISABLED)
   #error PKCS5_SHA384_SUPPORT parameter is not valid
#endif

//SHA-512 hash support
#ifndef PKCS5_SHA512_SUPPORT
   #define PKCS5_SHA512_SUPPORT ENABLED
#elif (PKCS5_SHA512_SUPPORT != ENABLED && PKCS5_SHA512_SUPPORT != DISABLED)
   #error PKCS5_SHA512_SUPPORT parameter is not valid
#endif

//SHA-512/224 hash support
#ifndef PKCS5_SHA512_224_SUPPORT
   #define PKCS5_SHA512_224_SUPPORT DISABLED
#elif (PKCS5_SHA512_224_SUPPORT != ENABLED && PKCS5_SHA512_224_SUPPORT != DISABLED)
   #error PKCS5_SHA512_224_SUPPORT parameter is not valid
#endif

//SHA-512/256 hash support
#ifndef PKCS5_SHA512_256_SUPPORT
   #define PKCS5_SHA512_256_SUPPORT DISABLED
#elif (PKCS5_SHA512_256_SUPPORT != ENABLED && PKCS5_SHA512_256_SUPPORT != DISABLED)
   #error PKCS5_SHA512_256_SUPPORT parameter is not valid
#endif

//SM3 hash support
#ifndef PKCS5_SM3_SUPPORT
   #define PKCS5_SM3_SUPPORT DISABLED
#elif (PKCS5_SM3_SUPPORT != ENABLED && PKCS5_SM3_SUPPORT != DISABLED)
   #error PKCS5_SM3_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Octet string
 **/

typedef struct
{
   const uint8_t *value;
   size_t length;
} Pkcs5OctetString;


/**
 * @brief PBES1 parameters
 **/

typedef struct
{
   Pkcs5OctetString salt;
   uint_t iterationCount;
} Pkcs5Pbes1Params;


/**
 * @brief Key derivation function
 **/

typedef struct
{
   Pkcs5OctetString kdfAlgoId;
   Pkcs5OctetString salt;
   uint_t iterationCount;
   uint_t keyLen;
   Pkcs5OctetString prfAlgoId;
} Pkcs5KeyDerivationFunc;


/**
 * @brief Encryption scheme
 **/

typedef struct
{
   Pkcs5OctetString oid;
   Pkcs5OctetString iv;
} Pkcs5EncryptionScheme;


/**
 * @brief PBES2 parameters
 **/

typedef struct
{
   Pkcs5KeyDerivationFunc keyDerivationFunc;
   Pkcs5EncryptionScheme encryptionScheme;
} Pkcs5Pbes2Params;


//PKCS #5 related constants
extern const uint8_t PBE_WITH_MD2_AND_DES_CBC_OID[9];
extern const uint8_t PBE_WITH_MD5_AND_DES_CBC_OID[9];
extern const uint8_t PBE_WITH_MD2_AND_RC2_CBC_OID[9];
extern const uint8_t PBE_WITH_MD5_AND_RC2_CBC_OID[9];
extern const uint8_t PBE_WITH_SHA1_AND_DES_CBC_OID[9];
extern const uint8_t PBE_WITH_SHA1_AND_RC2_CBC_OID[9];

extern const uint8_t PBES2_OID[9];

extern const uint8_t DES_CBC_OID[5];
extern const uint8_t DES_EDE3_CBC_OID[8];
extern const uint8_t AES128_CBC_OID[9];
extern const uint8_t AES192_CBC_OID[9];
extern const uint8_t AES256_CBC_OID[9];
extern const uint8_t CAMELLIA128_CBC_OID[11];
extern const uint8_t CAMELLIA192_CBC_OID[11];
extern const uint8_t CAMELLIA256_CBC_OID[11];
extern const uint8_t ARIA128_CBC_OID[9];
extern const uint8_t ARIA192_CBC_OID[9];
extern const uint8_t ARIA256_CBC_OID[9];
extern const uint8_t SM4_CBC_OID[8];

//PKCS #5 related functions
const HashAlgo *pkcs5GetPbes1HashAlgo(const uint8_t *oid, size_t length);
const HashAlgo *pkcs5GetPbes2HashAlgo(const uint8_t *oid, size_t length);

const CipherAlgo *pkcs5GetPbes1CipherAlgo(const uint8_t *oid, size_t length);
const CipherAlgo *pkcs5GetPbes2CipherAlgo(const uint8_t *oid, size_t length);

uint_t pkcs5GetPbes2KeyLength(const uint8_t *oid, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
