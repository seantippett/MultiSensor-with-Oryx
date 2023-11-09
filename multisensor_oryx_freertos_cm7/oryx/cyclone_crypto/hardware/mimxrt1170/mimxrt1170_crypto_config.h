/**
 * @file mimxrt1170_crypto_conifg.h
 * @brief CAAM-specific configuration file
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

#ifndef _MIMXRT1170_CRYPTO_CONFIG_H
#define _MIMXRT1170_CRYPTO_CONFIG_H

//Dependencies
#include "fsl_caam.h"

//CAAM-specific context (SHA-1)
#define SHA1_PRIVATE_CONTEXT \
   caam_handle_t caamHandle; \
   caam_hash_ctx_t caamContext;

//CAAM-specific context (SHA-256)
#define SHA256_PRIVATE_CONTEXT \
   caam_handle_t caamHandle; \
   caam_hash_ctx_t caamContext;

//CAAM-specific context (SHA-512)
#define SHA512_PRIVATE_CONTEXT \
   caam_handle_t caamHandle; \
   caam_hash_ctx_t caamContext;

#endif
