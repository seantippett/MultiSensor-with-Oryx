/**
 * @file mime.c
 * @brief MIME (Multipurpose Internet Mail Extensions)
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Eval.
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
#define TRACE_LEVEL HTTP_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "http/mime.h"
#include "debug.h"

//MIME type list
static const MimeType mimeTypeList[] =
{
   //Custom MIME types
   MIME_CUSTOM_TYPES

   //Text MIME types
   {".css",   "text/css"},
   {".csv",   "text/csv"},
   {".htc",   "text/x-component"},
   {".htm",   "text/html"},
   {".html",  "text/html"},
   {".shtm",  "text/html"},
   {".shtml", "text/html"},
   {".stm",   "text/html"},
   {".txt",   "text/plain"},
   {".vcf",   "text/vcard"},
   {".vcard", "text/vcard"},
   {".xml",   "text/xml"},

   //Image MIME types
   {".gif",   "image/gif"},
   {".ico",   "image/x-icon"},
   {".jpg",   "image/jpeg"},
   {".jpeg",  "image/jpeg"},
   {".png",   "image/png"},
   {".svg",   "image/svg+xml"},
   {".tif",   "image/tiff"},

   //Audio MIME types
   {".aac",   "audio/x-aac"},
   {".aif",   "audio/x-aiff"},
   {".mp3",   "audio/mpeg"},
   {".wav",   "audio/x-wav"},
   {".wma",   "audio/x-ms-wma"},

   //Video MIME types
   {".avi",   "video/x-msvideo"},
   {".flv",   "video/x-flv"},
   {".mov",   "video/quicktime"},
   {".mp4",   "video/mp4"},
   {".mpg",   "video/mpeg"},
   {".mpeg",  "video/mpeg"},
   {".wmv",   "video/x-ms-wmv"},

   //Application MIME types
   {".doc",   "application/msword"},
   {".gz",    "application/x-gzip"},
   {".gzip",  "application/x-gzip"},
   {".js",    "application/javascript"},
   {".json",  "application/json"},
   {".ogg",   "application/ogg"},
   {".pdf",   "application/pdf"},
   {".ppt",   "application/vnd.ms-powerpoint"},
   {".rar",   "application/x-rar-compressed"},
   {".rtf",   "application/rtf"},
   {".tar",   "application/x-tar"},
   {".tgz",   "application/x-gzip"},
   {".xht",   "application/xhtml+xml"},
   {".xhtml", "application/xhtml+xml"},
   {".xls",   "application/vnd.ms-excel"},
   {".zip",   "application/zip"}
};


/**
 * @brief Get the MIME type from a given extension
 *
 * This function translates a filename or a file extension into a MIME type
 *
 * @param[in] filename Filename from which to extract the MIME type
 * @return NULL-terminated string containing the associated MIME type
 **/

const char_t *mimeGetType(const char_t *filename)
{
   uint_t i;
   uint_t n;
   uint_t m;

   //MIME type for unknown extensions
   static const char_t defaultMimeType[] = "application/octet-stream";

   //Valid filename?
   if(filename != NULL)
   {
      //Get the length of the specified filename
      n = osStrlen(filename);

      //Search the MIME type that matches the specified extension
      for(i = 0; i < arraysize(mimeTypeList); i++)
      {
         //Length of the extension
         m = osStrlen(mimeTypeList[i].extension);
         //Compare file extensions
         if(m <= n && !osStrcasecmp(filename + n - m, mimeTypeList[i].extension))
            return mimeTypeList[i].type;
      }
   }

   //Return the default MIME type when an unknown extension is encountered
   return defaultMimeType;
}
