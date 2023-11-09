/**
 * @file x509_cert_ext_parse.h
 * @brief X.509 extension parsing
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

#ifndef _X509_CERT_EXT_PARSE_H
#define _X509_CERT_EXT_PARSE_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//X.509 related functions
error_t x509ParseCertExtensions(const uint8_t *data, size_t length,
   size_t *totalLength, X509Extensions *extensions, bool_t ignoreUnknown);

error_t x509ParseExtension(const uint8_t *data, size_t length,
   size_t *totalLength, X509Extension *extension);

error_t x509ParseBasicConstraints(bool_t critical, const uint8_t *data,
   size_t length, X509BasicConstraints *basicConstraints);

error_t x509ParseNameConstraints(bool_t critical, const uint8_t *data,
   size_t length, X509NameConstraints *nameConstraints);

error_t x509ParsePolicyConstraints(bool_t critical, const uint8_t *data,
   size_t length);

error_t x509ParsePolicyMappings(bool_t critical, const uint8_t *data,
   size_t length);

error_t x509ParseInhibitAnyPolicy(bool_t critical, const uint8_t *data,
   size_t length);

error_t x509ParseKeyUsage(bool_t critical, const uint8_t *data,
   size_t length, X509KeyUsage *keyUsage);

error_t x509ParseExtendedKeyUsage(bool_t critical, const uint8_t *data,
   size_t length, X509ExtendedKeyUsage *extKeyUsage);

error_t x509ParseSubjectAltName(bool_t critical, const uint8_t *data,
   size_t length, X509SubjectAltName *subjectAltName);

error_t x509ParseSubjectKeyId(bool_t critical, const uint8_t *data,
   size_t length, X509SubjectKeyId *subjectKeyId);

error_t x509ParseAuthKeyId(bool_t critical, const uint8_t *data,
   size_t length, X509AuthKeyId *authKeyId);

error_t x509ParseCrlDistrPoints(bool_t critical, const uint8_t *data,
   size_t length, X509CrlDistrPoints *crlDistrPoints);

error_t x509ParseDistrPoint(const uint8_t *data, size_t length,
   size_t *totalLength, X509DistrPoint *distrPoint);

error_t x509ParseDistrPointName(const uint8_t *data, size_t length,
   X509DistrPointName *distrPointName);

error_t x509ParseRelativeName(const uint8_t *data, size_t length,
   X509NameAttribute *relativeName);

error_t x509ParseReasonFlags(const uint8_t *data, size_t length,
   uint16_t *reasonFlags);

error_t x509ParseAuthInfoAccess(bool_t critical, const uint8_t *data,
   size_t length, X509AuthInfoAccess *authInfoAccess);

error_t x509ParseAccessDescription(const uint8_t *data, size_t length,
   size_t *totalLength, X509AccessDescription *accessDescription);

error_t x509ParsePkixOcspNoCheck(bool_t critical, const uint8_t *data,
   size_t length, X509PkixOcspNoCheck *pkixOcspNoCheck);

error_t x509ParseNsCertType(bool_t critical, const uint8_t *data,
   size_t length, X509NsCertType *nsCertType);

error_t x509ParseUnknownCertExtension(const uint8_t *oid,
   size_t oidLen, bool_t critical, const uint8_t *data, size_t dataLen,
   X509Extensions *extensions);

error_t x509CheckDuplicateExtension(const uint8_t *oid, size_t oidLen,
   const uint8_t *data, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
