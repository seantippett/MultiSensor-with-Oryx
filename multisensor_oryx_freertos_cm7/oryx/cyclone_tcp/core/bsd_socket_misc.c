/**
 * @file bsd_socket_misc.c
 * @brief Helper function for BSD socket API
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
#define TRACE_LEVEL BSD_SOCKET_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "core/bsd_socket.h"
#include "core/bsd_socket_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (BSD_SOCKET_SUPPORT == ENABLED)


/**
 * @brief Get first ancillary data header
 * @param[in] msg Pointer to the message header
 * @return Pointer to the first ancillary data header
 **/

struct cmsghdr *socketCmsgFirstHdr(struct msghdr *msg)
{
   CMSGHDR *first;

   //Initialize pointer
   first = NULL;

   //Check parameter
   if(msg != NULL)
   {
      //Check the length of the ancillary data buffer
      if(msg->msg_controllen >= sizeof(CMSGHDR))
      {
         //Point to the first ancillary data header
         first = (CMSGHDR *) msg->msg_control;
      }
   }

   //Return a pointer to the first ancillary data header
   return first;
}


/**
 * @brief Get next ancillary data header
 * @param[in] msg Pointer to the message header
 * @param[in] cmsg Pointer to the current ancillary data header
 * @return Pointer to the next ancillary data header
 **/

struct cmsghdr *socketCmsgNextHdr(struct msghdr *msg, struct cmsghdr *cmsg)
{
   size_t n;
   CMSGHDR *next;

   //Initialize pointer
   next = NULL;

   //Check parameters
   if(msg != NULL && cmsg != NULL)
   {
      //Valid ancillary data header?
      if(cmsg->cmsg_len >= sizeof(CMSGHDR) &&
         cmsg->cmsg_len <= msg->msg_controllen)
      {
         //Offset to the next ancillary data header
         n = (uint8_t *) cmsg - (uint8_t *) msg->msg_control +
            CMSG_ALIGN(cmsg->cmsg_len);

         //Check the length of the ancillary data buffer
         if((n + sizeof(CMSGHDR)) <= msg->msg_controllen)
         {
            //Point to the next ancillary data header
            next = (CMSGHDR *) ((uint8_t *) msg->msg_control + n);
         }
      }
   }

   //Return a pointer to the next ancillary data header
   return next;
}


/**
 * @brief Initializes a descriptor set
 * @param[in] fds Pointer to a descriptor set
 **/

void socketFdZero(fd_set *fds)
{
   //Reset the descriptor count
   fds->fd_count = 0;
}


/**
 * @brief Add a descriptor to an existing set
 * @param[in] fds Pointer to a descriptor set
 * @param[in] s Descriptor that identifies the socket to add
 **/

void socketFdSet(fd_set *fds, int_t s)
{
   int_t i;

   //Loop through descriptors
   for(i = 0; i < fds->fd_count; i++)
   {
      //The specified descriptor is already set?
      if(fds->fd_array[i] == s)
         return;
   }

   //Ensure the descriptor set is not full
   if(i < FD_SETSIZE)
   {
      //The specified descriptor can be safely added
      fds->fd_array[i] = s;
      //Adjust the size of the descriptor set
      fds->fd_count++;
   }
}


/**
 * @brief Remove a descriptor from an existing set
 * @param[in] fds Pointer to a descriptor set
 * @param[in] s Descriptor that identifies the socket to remove
 **/

void socketFdClr(fd_set *fds, int_t s)
{
   int_t i;
   int_t j;

   //Loop through descriptors
   for(i = 0; i < fds->fd_count; i++)
   {
      //Specified descriptor found?
      if(fds->fd_array[i] == s)
      {
         //Adjust the size of the descriptor set
         fds->fd_count--;

         //Remove the entry from the descriptor set
         for(j = i; j < fds->fd_count; j++)
         {
            fds->fd_array[j] = fds->fd_array[j + 1];
         }

         //Return immediately
         return;
      }
   }
}


/**
 * @brief Check whether a descriptor is set
 * @param[in] fds Pointer to a descriptor set
 * @param[in] s Descriptor that identifies the socket to test
 * @return Nonzero if s is a member of the set. Otherwise, zero
 **/

int_t socketFdIsSet(fd_set *fds, int_t s)
{
   int_t i;

   //Loop through descriptors
   for(i = 0; i < fds->fd_count; i++)
   {
      //Check whether the specified descriptor is set
      if(fds->fd_array[i] == s)
      {
         return TRUE;
      }
   }

   //The specified descriptor is not currently set
   return FALSE;
}


/**
 * @brief Set BSD error code
 * @param[in] socket Handle that identifies a socket
 * @param[in] errnoCode BSD error code
 **/

void socketSetErrnoCode(Socket *socket, uint_t errnoCode)
{
   //Valid socket handle?
   if(socket != NULL)
   {
      //Save error code
      socket->errnoCode = errnoCode;
   }

   //Save the code of the last error
   BSD_SOCKET_SET_ERRNO(errnoCode);
}


/**
 * @brief Translate error code
 * @param[in] socket Handle that identifies a socket
 * @param[in] errorCode Error code
 **/

void socketTranslateErrorCode(Socket *socket, error_t errorCode)
{
   uint_t errnoCode;

   //Translate error code
   switch(errorCode)
   {
   case NO_ERROR:
      errnoCode = 0;
      break;

   case ERROR_TIMEOUT:
      errnoCode = EWOULDBLOCK;
      break;

   case ERROR_INVALID_PARAMETER:
      errnoCode = EINVAL;
      break;

   case ERROR_CONNECTION_RESET:
      errnoCode = ECONNRESET;
      break;

   case ERROR_ALREADY_CONNECTED:
      errnoCode = EISCONN;
      break;

   case ERROR_NOT_CONNECTED:
      errnoCode = ENOTCONN;
      break;

   case ERROR_CONNECTION_CLOSING:
      errnoCode = ESHUTDOWN;
      break;

   case ERROR_CONNECTION_FAILED:
      errnoCode = ECONNREFUSED;
      break;

   default:
      errnoCode = EFAULT;
      break;
   }

   //Valid socket handle?
   if(socket != NULL)
   {
      //Save error code
      socket->errnoCode = errnoCode;
   }

   //Save the code of the last error
   BSD_SOCKET_SET_ERRNO(errnoCode);
}

#endif
