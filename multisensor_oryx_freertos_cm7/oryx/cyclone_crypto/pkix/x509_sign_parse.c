/**
 * @file x509_signature_parse.c
 * @brief RSA/DSA/ECDSA/EdDSA signature parsing
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
#include "pkix/x509_sign_parse.h"
#include "encoding/asn1.h"
#include "debug.h"

//Check crypto library configuration
#if (X509_SUPPORT == ENABLED)


/**
 * @brief Parse SignatureAlgorithm structure
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] totalLength Number of bytes that have been parsed
 * @param[out] signatureAlgo Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseSignatureAlgo(const uint8_t *data, size_t length,
   size_t *totalLength, X509SignAlgoId *signatureAlgo)
{
   error_t error;
   Asn1Tag tag;

   //Debug message
   TRACE_DEBUG("  Parsing SignatureAlgorithm...\r\n");

   //Read the contents of the SignatureAlgorithm structure
   error = asn1ReadSequence(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the total length of the field
   *totalLength = tag.totalLength;

   //Point to the first field of the sequence
   data = tag.value;
   length = tag.length;

   //Read the signature algorithm identifier
   error = asn1ReadOid(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the signature algorithm identifier
   signatureAlgo->oid.value = tag.value;
   signatureAlgo->oid.length = tag.length;

   //Point to the next field (if any)
   data += tag.totalLength;
   length -= tag.totalLength;

#if (X509_RSA_PSS_SUPPORT == ENABLED && RSA_SUPPORT == ENABLED)
   //RSASSA-PSS algorithm identifier?
   if(!asn1CheckOid(&tag, RSASSA_PSS_OID, sizeof(RSASSA_PSS_OID)))
   {
      //Read RSASSA-PSS parameters
      error = x509ParseRsaPssParameters(data, length,
         &signatureAlgo->rsaPssParams);
   }
   else
#endif
   //Unknown algorithm identifier?
   {
      //The parameters are optional
      error = NO_ERROR;
   }

   //Return status code
   return error;
}


/**
 * @brief Parse SignatureValue field
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] totalLength Number of bytes that have been parsed
 * @param[out] signature Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseSignatureValue(const uint8_t *data, size_t length,
   size_t *totalLength, X509OctetString *signature)
{
   error_t error;
   Asn1Tag tag;

   //Debug message
   TRACE_DEBUG("  Parsing SignatureValue...\r\n");

   //Read the contents of the SignatureValue structure
   error = asn1ReadTag(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the total length of the field
   *totalLength = tag.totalLength;

   //Enforce encoding, class and type
   error = asn1CheckTag(&tag, FALSE, ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_BIT_STRING);
   //Invalid tag?
   if(error)
      return error;

   //The bit string shall contain an initial octet which encodes the number
   //of unused bits in the final subsequent octet
   if(tag.length < 1 || tag.value[0] != 0x00)
      return ERROR_FAILURE;

   //Get the signature value
   signature->value = tag.value + 1;
   signature->length = tag.length - 1;

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief Parse RSASSA-PSS parameters
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] rsaPssParams Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseRsaPssParameters(const uint8_t *data, size_t length,
   X509RsaPssParameters *rsaPssParams)
{
   error_t error;
   Asn1Tag tag;

   //Clear RSASSA-PSS parameters
   osMemset(rsaPssParams, 0, sizeof(X509RsaPssParameters));

#if (SHA1_SUPPORT == ENABLED)
   //The default hash algorithm is SHA-1 (refer to RFC 4055, section 3.1)
   rsaPssParams->hashAlgo.value = SHA1_OID;
   rsaPssParams->hashAlgo.length = sizeof(SHA1_OID);
#endif

#if (RSA_SUPPORT == ENABLED)
   //The default mask generation function is MGF1 with SHA-1
   rsaPssParams->maskGenAlgo.value = MGF1_OID;
   rsaPssParams->maskGenAlgo.length = sizeof(MGF1_OID);
#endif

#if (SHA1_SUPPORT == ENABLED)
   //MGF1 requires a one-way hash function that is identified in the
   //parameters field of the MGF1 algorithm identifier
   rsaPssParams->maskGenHashAlgo.value = SHA1_OID;
   rsaPssParams->maskGenHashAlgo.length = sizeof(SHA1_OID);
#endif

   //The default length of the salt is 20
   rsaPssParams->saltLen = 20;

   //Read the contents of the structure
   error = asn1ReadSequence(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Point to the first field of the sequence
   data = tag.value;
   length = tag.length;

   //Parse RSASSA-PSS parameters
   while(length > 0)
   {
      //Read current parameter
      error = asn1ReadTag(data, length, &tag);
      //Failed to decode ASN.1 tag?
      if(error)
         return error;

      //The tags in this sequence are explicit
      if(!asn1CheckTag(&tag, TRUE, ASN1_CLASS_CONTEXT_SPECIFIC, 0))
      {
         //Parse hashAlgorithm parameter
         error = x509ParseRsaPssHashAlgo(tag.value, tag.length,
            rsaPssParams);
      }
      else if(!asn1CheckTag(&tag, TRUE, ASN1_CLASS_CONTEXT_SPECIFIC, 1))
      {
         //Parse maskGenAlgorithm parameter
         error = x509ParseRsaPssMaskGenAlgo(tag.value, tag.length,
            rsaPssParams);
      }
      else if(!asn1CheckTag(&tag, TRUE, ASN1_CLASS_CONTEXT_SPECIFIC, 2))
      {
         //Parse saltLength parameter
         error = x509ParseRsaPssSaltLength(tag.value, tag.length,
            rsaPssParams);
      }
      else
      {
         //Discard current parameter
         error = NO_ERROR;
      }

      //Any parsing error?
      if(error)
         return error;

      //Next parameter
      data += tag.totalLength;
      length -= tag.totalLength;
   }

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief Parse RSASSA-PSS hash algorithm
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] rsaPssParams Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseRsaPssHashAlgo(const uint8_t *data, size_t length,
   X509RsaPssParameters *rsaPssParams)
{
   error_t error;
   Asn1Tag tag;

   //Read the contents of the structure
   error = asn1ReadSequence(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Point to the first field of the sequence
   data = tag.value;
   length = tag.length;

   //Read hash algorithm identifier
   error = asn1ReadOid(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the hash algorithm identifier
   rsaPssParams->hashAlgo.value = tag.value;
   rsaPssParams->hashAlgo.length = tag.length;

   //No error to report
   return NO_ERROR;
}


/**
 * @brief Parse RSASSA-PSS mask generation algorithm
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] rsaPssParams Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseRsaPssMaskGenAlgo(const uint8_t *data, size_t length,
   X509RsaPssParameters *rsaPssParams)
{
   error_t error;
   Asn1Tag tag;

   //Read the contents of the structure
   error = asn1ReadSequence(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Point to the first field of the sequence
   data = tag.value;
   length = tag.length;

   //Read mask generation algorithm identifier
   error = asn1ReadOid(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the mask generation algorithm identifier
   rsaPssParams->maskGenAlgo.value = tag.value;
   rsaPssParams->maskGenAlgo.length = tag.length;

   //Point to the next field
   data += tag.totalLength;
   length -= tag.totalLength;

   //Read the algorithm identifier of the one-way hash function employed
   //with the mask generation function
   error = x509ParseRsaPssMaskGenHashAlgo(data, length, rsaPssParams);
   //Any error to report?
   if(error)
      return error;

   //No error to report
   return NO_ERROR;
}


/**
 * @brief Parse RSASSA-PSS mask generation hash algorithm
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] rsaPssParams Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseRsaPssMaskGenHashAlgo(const uint8_t *data, size_t length,
   X509RsaPssParameters *rsaPssParams)
{
   error_t error;
   Asn1Tag tag;

   //Read the contents of the structure
   error = asn1ReadSequence(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Point to the first field of the sequence
   data = tag.value;
   length = tag.length;

   //Read the algorithm identifier of the one-way hash function employed
   //with the mask generation function
   error = asn1ReadOid(data, length, &tag);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Save the hash algorithm identifier
   rsaPssParams->maskGenHashAlgo.value = tag.value;
   rsaPssParams->maskGenHashAlgo.length = tag.length;

   //No error to report
   return NO_ERROR;
}


/**
 * @brief Parse RSASSA-PSS salt length
 * @param[in] data Pointer to the ASN.1 structure to parse
 * @param[in] length Length of the ASN.1 structure
 * @param[out] rsaPssParams Information resulting from the parsing process
 * @return Error code
 **/

error_t x509ParseRsaPssSaltLength(const uint8_t *data, size_t length,
   X509RsaPssParameters *rsaPssParams)
{
   error_t error;
   int32_t saltLen;
   Asn1Tag tag;

   //Read the saltLength field
   error = asn1ReadInt32(data, length, &tag, &saltLen);
   //Failed to decode ASN.1 tag?
   if(error)
      return error;

   //Sanity check
   if(saltLen < 0)
      return ERROR_INVALID_SYNTAX;

   //Save the length of the salt
   rsaPssParams->saltLen = (size_t) saltLen;

   //No error to report
   return NO_ERROR;
}

#endif
