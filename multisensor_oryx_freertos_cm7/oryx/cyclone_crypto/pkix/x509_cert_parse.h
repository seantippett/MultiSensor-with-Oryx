/**
 * @file x509_cert_parse.h
 * @brief X.509 certificate parsing
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

#ifndef _X509_CERT_PARSE_H
#define _X509_CERT_PARSE_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//X.509 related functions
error_t x509ParseCertificate(const uint8_t *data, size_t length,
   X509CertInfo *certInfo);

error_t x509ParseCertificateEx(const uint8_t *data, size_t length,
   X509CertInfo *certInfo, bool_t ignoreUnknown);

error_t x509ParseTbsCertificate(const uint8_t *data, size_t length,
   size_t *totalLength, X509TbsCertificate *tbsCert, bool_t ignoreUnknown);

error_t x509ParseVersion(const uint8_t *data, size_t length,
   size_t *totalLength, X509Version *version);

error_t x509ParseSerialNumber(const uint8_t *data, size_t length,
   size_t *totalLength, X509SerialNumber *serialNumber);

error_t x509ParseIssuerUniqueId(const uint8_t *data, size_t length,
   size_t *totalLength);

error_t x509ParseSubjectUniqueId(const uint8_t *data, size_t length,
   size_t *totalLength);

error_t x509ParseName(const uint8_t *data, size_t length,
   size_t *totalLength, X509Name *name);

error_t x509ParseNameAttribute(const uint8_t *data, size_t length,
   size_t *totalLength, X509NameAttribute *nameAttribute);

error_t x509ParseGeneralNames(const uint8_t *data, size_t length,
   X509GeneralName *generalNames, uint_t maxGeneralNames,
   uint_t *numGeneralNames);

error_t x509ParseGeneralName(const uint8_t *data, size_t length,
   size_t *totalLength, X509GeneralName *generalName);

error_t x509ParseGeneralSubtrees(const uint8_t *data, size_t length);

error_t x509ParseGeneralSubtree(const uint8_t *data, size_t length,
   size_t *totalLength, X509GeneralName *generalName);

error_t x509ParseValidity(const uint8_t *data, size_t length,
   size_t *totalLength, X509Validity *validity);

error_t x509ParseTime(const uint8_t *data, size_t length,
   size_t *totalLength, DateTime *dateTime);

error_t x509ParseInt(const uint8_t *data, size_t length, uint_t *value);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
