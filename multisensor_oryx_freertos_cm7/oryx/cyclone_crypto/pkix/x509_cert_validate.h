/**
 * @file x509_cert_validate.h
 * @brief X.509 certificate validation
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

#ifndef _X509_CERT_VALIDATE_H
#define _X509_CERT_VALIDATE_H

//Dependencies
#include "core/crypto.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//X.509 related functions
error_t x509ValidateCertificate(const X509CertInfo *certInfo,
   const X509CertInfo *issuerCertInfo, uint_t pathLen);

error_t x509CheckSubjectName(const X509CertInfo *certInfo,
   const char_t *fqdn);

error_t x509CheckNameConstraints(const char_t *subjectName,
   const X509CertInfo *certInfo);

bool_t x509CompareName(const uint8_t *name1, size_t nameLen1,
   const uint8_t *name2, size_t nameLen2);

bool_t x509CompareSubjectName(const char_t *subjectName, size_t subjectNameLen,
   const char_t *fqdn);

bool_t x509CompareSubtree(const char_t *subjectName, const char_t *subtree,
   size_t subtreeLen);

bool_t x509CompareIpAddr(const uint8_t *ipAddr, size_t ipAddrLen,
   const char_t *str);

error_t x509ParseIpv4Addr(const char_t *str, uint8_t *ipAddr);
error_t x509ParseIpv6Addr(const char_t *str, uint8_t *ipAddr);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
