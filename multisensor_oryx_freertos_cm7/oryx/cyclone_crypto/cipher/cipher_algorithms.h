/**
 * @file cipher_algorithms.h
 * @brief Collection of cipher algorithms
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

#ifndef _CIPHER_ALGORITHMS_H
#define _CIPHER_ALGORITHMS_H

//Dependencies
#include "core/crypto.h"

//RC2 cipher support?
#if (RC2_SUPPORT == ENABLED)
   #include "cipher/rc2.h"
#endif

//RC4 cipher support?
#if (RC4_SUPPORT == ENABLED)
   #include "cipher/rc4.h"
#endif

//RC6 cipher support?
#if (RC6_SUPPORT == ENABLED)
   #include "cipher/rc6.h"
#endif

//CAST-128 cipher support?
#if (CAST128_SUPPORT == ENABLED)
   #include "cipher/cast128.h"
#endif

//CAST-256 cipher support?
#if (CAST256_SUPPORT == ENABLED)
   #include "cipher/cast256.h"
#endif

//IDEA cipher support?
#if (IDEA_SUPPORT == ENABLED)
   #include "cipher/idea.h"
#endif

//DES cipher support?
#if (DES_SUPPORT == ENABLED)
   #include "cipher/des.h"
#endif

//Triple DES cipher support?
#if (DES3_SUPPORT == ENABLED)
   #include "cipher/des3.h"
#endif

//AES cipher support?
#if (AES_SUPPORT == ENABLED)
   #include "cipher/aes.h"
#endif

//Blowfish cipher support?
#if (BLOWFISH_SUPPORT == ENABLED)
   #include "cipher/blowfish.h"
#endif

//Twofish cipher support?
#if (TWOFISH_SUPPORT == ENABLED)
   #include "cipher/twofish.h"
#endif

//MARS cipher support?
#if (MARS_SUPPORT == ENABLED)
   #include "cipher/mars.h"
#endif

//Serpent cipher support?
#if (SERPENT_SUPPORT == ENABLED)
   #include "cipher/serpent.h"
#endif

//Camellia cipher support?
#if (CAMELLIA_SUPPORT == ENABLED)
   #include "cipher/camellia.h"
#endif

//ARIA cipher support?
#if (ARIA_SUPPORT == ENABLED)
   #include "cipher/aria.h"
#endif

//SEED cipher support?
#if (SEED_SUPPORT == ENABLED)
   #include "cipher/seed.h"
#endif

//SM4 cipher support?
#if (SM4_SUPPORT == ENABLED)
   #include "cipher/sm4.h"
#endif

//PRESENT cipher support?
#if (PRESENT_SUPPORT == ENABLED)
   #include "cipher/present.h"
#endif

//TEA cipher support?
#if (TEA_SUPPORT == ENABLED)
   #include "cipher/tea.h"
#endif

//XTEA cipher support?
#if (XTEA_SUPPORT == ENABLED)
   #include "cipher/xtea.h"
#endif

//Trivium cipher support?
#if (TRIVIUM_SUPPORT == ENABLED)
   #include "cipher/trivium.h"
#endif

//Salsa20 cipher support?
#if (SALSA20_SUPPORT == ENABLED)
   #include "cipher/salsa20.h"
#endif

//ChaCha cipher support?
#if (CHACHA_SUPPORT == ENABLED)
   #include "cipher/chacha.h"
#endif

//Maximum block size
#if (RC6_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE RC6_BLOCK_SIZE
#elif (CAST256_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE CAST256_BLOCK_SIZE
#elif (AES_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE AES_BLOCK_SIZE
#elif (TWOFISH_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE TWOFISH_BLOCK_SIZE
#elif (MARS_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE MARS_BLOCK_SIZE
#elif (SERPENT_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE SERPENT_BLOCK_SIZE
#elif (CAMELLIA_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE CAMELLIA_BLOCK_SIZE
#elif (ARIA_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE ARIA_BLOCK_SIZE
#elif (SEED_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE SEED_BLOCK_SIZE
#elif (SM4_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE SM4_BLOCK_SIZE
#elif (RC2_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE RC2_BLOCK_SIZE
#elif (CAST128_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE CAST128_BLOCK_SIZE
#elif (IDEA_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE IDEA_BLOCK_SIZE
#elif (DES_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE DES_BLOCK_SIZE
#elif (DES3_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE DES3_BLOCK_SIZE
#elif (BLOWFISH_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE BLOWFISH_BLOCK_SIZE
#elif (PRESENT_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE PRESENT_BLOCK_SIZE
#elif (TEA_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE TEA_BLOCK_SIZE
#elif (XTEA_SUPPORT == ENABLED)
   #define MAX_CIPHER_BLOCK_SIZE XTEA_BLOCK_SIZE
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Generic cipher algorithm context
 **/

typedef union
{
#if (RC2_SUPPORT == ENABLED)
   Rc2Context rc2Context;
#endif
#if (RC4_SUPPORT == ENABLED)
   Rc4Context rc4Context;
#endif
#if (RC6_SUPPORT == ENABLED)
   Rc6Context rc6Context;
#endif
#if (CAST128_SUPPORT == ENABLED)
   Cast128Context cast128Context;
#endif
#if (CAST256_SUPPORT == ENABLED)
   Cast256Context cast256Context;
#endif
#if (IDEA_SUPPORT == ENABLED)
   IdeaContext ideaContext;
#endif
#if (DES_SUPPORT == ENABLED)
   DesContext desContext;
#endif
#if (DES3_SUPPORT == ENABLED)
   Des3Context des3Context;
#endif
#if (AES_SUPPORT == ENABLED)
   AesContext aesContext;
#endif
#if (BLOWFISH_SUPPORT == ENABLED)
   BlowfishContext blowfishContext;
#endif
#if (TWOFISH_SUPPORT == ENABLED)
   TwofishContext twofishContext;
#endif
#if (MARS_SUPPORT == ENABLED)
   MarsContext marsContext;
#endif
#if (SERPENT_SUPPORT == ENABLED)
   SerpentContext serpentContext;
#endif
#if (CAMELLIA_SUPPORT == ENABLED)
   CamelliaContext camelliaContext;
#endif
#if (ARIA_SUPPORT == ENABLED)
   AriaContext ariaContext;
#endif
#if (SEED_SUPPORT == ENABLED)
   SeedContext seedContext;
#endif
#if (SM4_SUPPORT == ENABLED)
   Sm4Context sm4Context;
#endif
#if (PRESENT_SUPPORT == ENABLED)
   PresentContext presentContext;
#endif
#if (TEA_SUPPORT == ENABLED)
   TeaContext teaContext;
#endif
#if (XTEA_SUPPORT == ENABLED)
   XteaContext xteaContext;
#endif
#if (TRIVIUM_SUPPORT == ENABLED)
   TriviumContext triviumContext;
#endif
} CipherContext;


//C++ guard
#ifdef __cplusplus
}
#endif

#endif
