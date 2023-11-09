/**
 * @file crypto_legacy.h
 * @brief Legacy definitions
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

#ifndef _CRYPTO_LEGACY_H
#define _CRYPTO_LEGACY_H

//Deprecated functions
#define mpiReadRaw(r, data, length) mpiImport(r, data, length, MPI_FORMAT_BIG_ENDIAN)
#define mpiWriteRaw(a, data, length) mpiExport(a, data, length, MPI_FORMAT_BIG_ENDIAN)

#ifdef CURVE25519_SUPPORT
   #define X25519_SUPPORT CURVE25519_SUPPORT
#endif

#ifdef CURVE448_SUPPORT
   #define X448_SUPPORT CURVE448_SUPPORT
#endif

#define ecdsaGenerateKeyPair ecGenerateKeyPair
#define ecdsaGeneratePrivateKey ecGeneratePrivateKey
#define ecdsaGeneratePublicKey ecGeneratePublicKey

#define MAX_HASH_CONTEXT_SIZE sizeof(HashContext)
#define MAX_CIPHER_CONTEXT_SIZE sizeof(CipherContext)

#ifdef SAMD51_CRYPTO_PUKCC_SUPPORT
   #define SAMD51_CRYPTO_PKC_SUPPORT SAMD51_CRYPTO_PUKCC_SUPPORT
#endif

#ifdef SAME54_CRYPTO_PUKCC_SUPPORT
   #define SAME54_CRYPTO_PKC_SUPPORT SAME54_CRYPTO_PUKCC_SUPPORT
#endif

#define yarrowRelease yarrowDeinit

#define X509CertificateInfo X509CertInfo
#define X509SignatureAlgoId X509SignAlgoId

#endif
