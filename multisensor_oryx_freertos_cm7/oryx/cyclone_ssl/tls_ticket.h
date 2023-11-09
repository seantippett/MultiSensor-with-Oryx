/**
 * @file tls_ticket.h
 * @brief TLS session tickets
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

#ifndef _TLS_TICKET_H
#define _TLS_TICKET_H

//Dependencies
#include "tls.h"
#include "cipher/aes.h"
#include "aead/gcm.h"

//Size of ticket key names
#ifndef TLS_TICKET_KEY_NAME_SIZE
   #define TLS_TICKET_KEY_NAME_SIZE 16
#elif (TLS_TICKET_KEY_NAME_SIZE < 1)
   #error TLS_TICKET_KEY_NAME_SIZE parameter is not valid
#endif

//Size of ticket keys
#ifndef TLS_TICKET_KEY_SIZE
   #define TLS_TICKET_KEY_SIZE 32
#elif (TLS_TICKET_KEY_SIZE < 1)
   #error TLS_TICKET_KEY_SIZE parameter is not valid
#endif

//Size of ticket IVs
#ifndef TLS_TICKET_IV_SIZE
   #define TLS_TICKET_IV_SIZE 12
#elif (TLS_TICKET_IV_SIZE < 1)
   #error TLS_TICKET_IV_SIZE parameter is not valid
#endif

//Size of ticket authentication tags
#ifndef TLS_TICKET_TAG_SIZE
   #define TLS_TICKET_TAG_SIZE 16
#elif (TLS_TICKET_TAG_SIZE < 1)
   #error TLS_TICKET_TAG_SIZE parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Session ticket encryption state
 **/

typedef struct
{
   bool_t valid;                              ///<Valid set of keys
   systime_t timestamp;                       ///<Generation time
   uint8_t keyName[TLS_TICKET_KEY_NAME_SIZE]; ///<Key identifier
   uint8_t key[TLS_TICKET_KEY_SIZE];          ///<Encryption key
} TlsTicketEncryptionState;


/**
 * @brief Session ticket encryption context
 **/

typedef struct
{
   OsMutex mutex;                                ///<Mutex preventing simultaneous access to the context
   TlsTicketEncryptionState encryptionState;     ///<Current set of keys
   TlsTicketEncryptionState prevEncryptionState; ///<Previous set of keys
   AesContext aesContext;                        ///<AES context
   GcmContext gcmContext;                        ///<GCM context
} TlsTicketContext;


//TLS related functions
error_t tlsInitTicketContext(TlsTicketContext *ticketContext);

error_t tlsEncryptTicket(TlsContext *context, const uint8_t *plaintext,
   size_t plaintextLen, uint8_t *ciphertext, size_t *ciphertextLen, void *param);

error_t tlsDecryptTicket(TlsContext *context, const uint8_t *ciphertext,
   size_t ciphertextLen, uint8_t *plaintext, size_t *plaintextLen, void *param);

error_t tlsGenerateTicketKeys(TlsTicketContext *ticketContext,
   const PrngAlgo *prngAlgo, void *prngContext);

void tlsCheckTicketKeyLifetime(TlsTicketEncryptionState *state);

bool_t tlsCompareTicketKeyName(const uint8_t *ticket, size_t ticketLen,
   const TlsTicketEncryptionState *state);

void tlsFreeTicketContext(TlsTicketContext *ticketContext);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
