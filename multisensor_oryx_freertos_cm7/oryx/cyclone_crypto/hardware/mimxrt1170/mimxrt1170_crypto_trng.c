/**
 * @file mimxrt1170_crypto_trng.c
 * @brief i.MX RT1170 true random number generator
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

//Switch to the appropriate trace level
#define TRACE_LEVEL CRYPTO_TRACE_LEVEL

//Dependencies
#include "fsl_device_registers.h"
#include "fsl_caam.h"
#include "core/crypto.h"
#include "hardware/mimxrt1170/mimxrt1170_crypto.h"
#include "hardware/mimxrt1170/mimxrt1170_crypto_trng.h"
#include "debug.h"

//Check crypto library configuration
#if (MIMXRT1170_CRYPTO_TRNG_SUPPORT == ENABLED)


/**
 * @brief Get random data from the TRNG module
 * @param[out] data Buffer where to store random data
 * @param[in] length Number of random bytes to generate
 **/

error_t trngGetRandomData(uint8_t *data, size_t length)
{
   status_t status;
   caam_handle_t caamHandle;

   //Acquire exclusive access to the CAAM module
   osAcquireMutex(&mimxrt1170CryptoMutex);

   //Set CAAM job ring
   caamHandle.jobRing = kCAAM_JobRing0;

   //Generate random data
   status = CAAM_RNG_GetRandomData(CAAM, &caamHandle, kCAAM_RngStateHandle0,
      data, length, kCAAM_RngDataAny, NULL);

   //Release exclusive access to the CAAM module
   osReleaseMutex(&mimxrt1170CryptoMutex);

   //Return status code
   return (status == kStatus_Success) ? NO_ERROR : ERROR_FAILURE;
}

#endif
