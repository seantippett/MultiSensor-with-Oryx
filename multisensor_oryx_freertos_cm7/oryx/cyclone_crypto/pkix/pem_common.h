/**
 * @file pem_common.h
 * @brief PEM common definitions
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

#ifndef _PEM_COMMON_H
#define _PEM_COMMON_H

//Dependencies
#include "core/crypto.h"

//Encrypted private key support
#ifndef PEM_ENCRYPTED_KEY_SUPPORT
   #define PEM_ENCRYPTED_KEY_SUPPORT DISABLED
#elif (PEM_ENCRYPTED_KEY_SUPPORT != ENABLED && PEM_ENCRYPTED_KEY_SUPPORT != DISABLED)
   #error PEM_ENCRYPTED_KEY_SUPPORT parameter is not valid
#endif

//DES encryption support (insecure)
#ifndef PEM_DES_SUPPORT
   #define PEM_DES_SUPPORT DISABLED
#elif (PEM_DES_SUPPORT != ENABLED && PEM_DES_SUPPORT != DISABLED)
   #error PEM_DES_SUPPORT parameter is not valid
#endif

//Triple DES encryption support (weak)
#ifndef PEM_3DES_SUPPORT
   #define PEM_3DES_SUPPORT DISABLED
#elif (PEM_3DES_SUPPORT != ENABLED && PEM_3DES_SUPPORT != DISABLED)
   #error PEM_3DES_SUPPORT parameter is not valid
#endif

//AES encryption support
#ifndef PEM_AES_SUPPORT
   #define PEM_AES_SUPPORT ENABLED
#elif (PEM_AES_SUPPORT != ENABLED && PEM_AES_SUPPORT != DISABLED)
   #error PEM_AES_SUPPORT parameter is not valid
#endif

//Camellia cipher support?
#ifndef PEM_CAMELLIA_SUPPORT
   #define PEM_CAMELLIA_SUPPORT DISABLED
#elif (PEM_CAMELLIA_SUPPORT != ENABLED && PEM_CAMELLIA_SUPPORT != DISABLED)
   #error PEM_CAMELLIA_SUPPORT parameter is not valid
#endif

//ARIA cipher support?
#ifndef PEM_ARIA_SUPPORT
   #define PEM_ARIA_SUPPORT DISABLED
#elif (PEM_ARIA_SUPPORT != ENABLED && PEM_ARIA_SUPPORT != DISABLED)
   #error PEM_ARIA_SUPPORT parameter is not valid
#endif

//SM4 encryption support
#ifndef PEM_SM4_SUPPORT
   #define PEM_SM4_SUPPORT DISABLED
#elif (PEM_SM4_SUPPORT != ENABLED && PEM_SM4_SUPPORT != DISABLED)
   #error PEM_SM4_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief String representation
 **/

typedef struct
{
   const char_t *value;
   size_t length;
} PemString;


/**
 * @brief "Proc-Type" header field
 **/

typedef struct
{
   PemString version;
   PemString type;
} PemProcType;


/**
 * @brief "DEK-Info" header field
 **/

typedef struct
{
   PemString algo;
   PemString iv;
} PemDekInfo;


/**
 * @brief PEM encapsulated header
 **/

typedef struct
{
   PemProcType procType;
   PemDekInfo dekInfo;
} PemHeader;


//PEM related functions
error_t pemDecodeFile(const char_t *input, size_t inputLen, const char_t *label,
   uint8_t *output, size_t *outputLen, PemHeader *header, size_t *consumed);

error_t pemEncodeFile(const void *input, size_t inputLen, const char_t *label,
   char_t *output, size_t *outputLen);

error_t pemParseHeader(const char_t *input, size_t inputLen,
   PemHeader *header, size_t *consumed);

void pemParseHeaderField(PemString *line, PemHeader *header);

int_t pemFindTag(const char_t *input, size_t inputLen, const char_t *tag1,
   const char_t *tag2, const char_t *tag3);

int_t pemFindChar(const PemString *s, char_t c);
bool_t pemCompareString(const PemString *string, const char_t *value);
bool_t pemTokenizeString(PemString *s, char_t c, PemString *token);
void pemTrimWhitespace(PemString *s);

const CipherAlgo *pemGetCipherAlgo(const PemString *algo);
uint_t pemGetKeyLength(const PemString *algo);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
