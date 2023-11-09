/**
 * @file dhcp_common.c
 * @brief Definitions common to DHCP client and server
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
#define TRACE_LEVEL DHCP_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "dhcp/dhcp_client.h"
#include "dhcp/dhcp_server.h"
#include "dhcp/dhcp_common.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (IPV4_SUPPORT == ENABLED && (DHCP_CLIENT_SUPPORT == ENABLED || \
   DHCP_SERVER_SUPPORT == ENABLED))


/**
 * @brief Append an option to a DHCP message
 * @param[in] message Pointer to the DHCP message
 * @param[in,out] messageLen Actual length of the DHCP message
 * @param[in] optionCode Option code
 * @param[in] optionValue Option value
 * @param[in] optionLen Length of the option value
 * @return Error code
 **/

error_t dhcpAddOption(DhcpMessage *message, size_t *messageLen,
   uint8_t optionCode, const void *optionValue, size_t optionLen)
{
   size_t n;
   DhcpOption *option;

   //Check parameters
   if(message == NULL || messageLen == NULL)
      return ERROR_INVALID_PARAMETER;

   //Check the length of the DHCP message
   if(*messageLen < (sizeof(DhcpMessage) + sizeof(uint8_t)))
      return ERROR_INVALID_LENGTH;

   //Check the length of the option
   if(optionLen > 0 && optionValue == NULL)
      return ERROR_INVALID_PARAMETER;

   if(optionLen > UINT8_MAX)
      return ERROR_INVALID_LENGTH;

   //Ensure that the length of the resulting message will not exceed the
   //maximum DHCP message size
   if((*messageLen + sizeof(DhcpOption) + optionLen) > DHCP_MAX_MSG_SIZE)
      return ERROR_BUFFER_OVERFLOW;

   //Retrieve the total length of the options field, excluding the end option
   n = *messageLen - sizeof(DhcpMessage) - sizeof(uint8_t);

   //Point to the buffer where to format the option
   option = (DhcpOption *) (message->options + n);

   //Set option code
   option->code = optionCode;
   //Set option length
   option->length = (uint8_t) optionLen;
   //Copy option value
   osMemcpy(option->value, optionValue, optionLen);

   //Determine the length of the options field
   n += sizeof(DhcpOption) + option->length;

   //Always terminate the options field with 255
   message->options[n++] = DHCP_OPT_END;

   //Update the length of the DHCPv6 message
   *messageLen = sizeof(DhcpMessage) + n;

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief Search a DHCP message for a given option
 * @param[in] message Pointer to the DHCP message
 * @param[in] length Length of the message
 * @param[in] optionCode Code of the option to find
 * @return If the specified option is found, a pointer to the corresponding
 *   option is returned. Otherwise NULL pointer is returned
 **/

DhcpOption *dhcpGetOption(const DhcpMessage *message, size_t length,
   uint8_t optionCode)
{
   size_t i;
   DhcpOption *option;

   //Make sure the DHCP header is valid
   if(length >= sizeof(DhcpMessage))
   {
      //Get the length of the options field
      length -= sizeof(DhcpMessage);

      //Loop through the list of options
      for(i = 0; i < length; i++)
      {
         //Point to the current option
         option = (DhcpOption *) (message->options + i);

         //Check option code
         if(option->code == DHCP_OPT_PAD)
         {
            //The pad option can be used to cause subsequent fields to align
            //on word boundaries
         }
         else if(option->code == DHCP_OPT_END)
         {
            //The end option marks the end of valid information in the vendor
            //field
            break;
         }
         else
         {
            //The option code is followed by a one-byte length field
            if((i + 1) >= length)
            {
               break;
            }

            //Check the length of the option
            if((i + sizeof(DhcpOption) + option->length) > length)
            {
               break;
            }

            //Matching option code?
            if(option->code == optionCode)
            {
               return option;
            }

            //Jump to the next option
            i += option->length + 1;
         }
      }
   }

   //The specified option code was not found
   return NULL;
}

#endif
