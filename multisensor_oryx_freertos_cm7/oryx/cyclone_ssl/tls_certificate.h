/**
 * @file tls_certificate.h
 * @brief X.509 certificate handling
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

#ifndef _TLS_CERTIFICATE_H
#define _TLS_CERTIFICATE_H

//Dependencies
#include "tls.h"
#include "pkix/x509_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsFormatCertificateList(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsFormatRawPublicKey(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsParseCertificateList(TlsContext *context, const uint8_t *p,
   size_t length);

error_t tlsParseRawPublicKey(TlsContext *context, const uint8_t *p,
   size_t length);

bool_t tlsIsCertificateAcceptable(TlsContext *context, const TlsCertDesc *cert,
   const uint8_t *certTypes, size_t numCertTypes, const TlsSignHashAlgos *signHashAlgos,
   const TlsSignHashAlgos *certSignHashAlgos, const TlsSupportedGroupList *curveList,
   const TlsCertAuthorities *certAuthorities);

error_t tlsValidateCertificate(TlsContext *context,
   const X509CertInfo *certInfo, uint_t pathLen,
   const char_t *subjectName);

error_t tlsGetCertificateType(const X509CertInfo *certInfo,
   TlsCertificateType *certType, TlsNamedGroup *namedCurve);

error_t tlsGetCertificateSignAlgo(const X509CertInfo *certInfo,
   TlsSignatureAlgo *signAlgo, TlsHashAlgo *hashAlgo);

error_t tlsReadSubjectPublicKey(TlsContext *context,
   const X509SubjectPublicKeyInfo *subjectPublicKeyInfo);

error_t tlsCheckKeyUsage(const X509CertInfo *certInfo,
   TlsConnectionEnd entity, TlsKeyExchMethod keyExchMethod);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
