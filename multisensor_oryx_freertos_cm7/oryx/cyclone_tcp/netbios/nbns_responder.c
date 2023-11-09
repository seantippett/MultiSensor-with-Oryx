/**
 * @file nbns_responder.c
 * @brief NBNS responder (NetBIOS Name Service)
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
#define TRACE_LEVEL NBNS_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "netbios/nbns_responder.h"
#include "netbios/nbns_common.h"
#include "dns/dns_debug.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (NBNS_RESPONDER_SUPPORT == ENABLED && IPV4_SUPPORT == ENABLED)


/**
 * @brief Process NBNS query message
 * @param[in] interface Underlying network interface
 * @param[in] pseudoHeader UDP pseudo header
 * @param[in] udpHeader UDP header
 * @param[in] message Pointer to the NBNS query message
 * @param[in] length Length of the message
 **/

void nbnsProcessQuery(NetInterface *interface, const Ipv4PseudoHeader *pseudoHeader,
   const UdpHeader *udpHeader, const NbnsHeader *message, size_t length)
{
   size_t pos;
   DnsQuestion *question;

   //The NBNS query shall contain one question
   if(ntohs(message->qdcount) != 1)
      return;

   //Parse NetBIOS name
   pos = nbnsParseName(message, length, sizeof(DnsHeader), NULL);

   //Invalid name?
   if(!pos)
      return;
   //Malformed NBNS query message?
   if((pos + sizeof(DnsQuestion)) > length)
      return;

   //Point to the corresponding entry
   question = DNS_GET_QUESTION(message, pos);

   //Check the class and the type of the request
   if(ntohs(question->qclass) != DNS_RR_CLASS_IN)
      return;
   if(ntohs(question->qtype) != DNS_RR_TYPE_NB)
      return;

   //Compare NetBIOS names
   if(nbnsCompareName(message, length, sizeof(DnsHeader), interface->hostname))
   {
      uint16_t destPort;
      IpAddr destIpAddr;

      //A response packet is always sent to the source UDP port and source IP
      //address of the request packet
      destIpAddr.length = sizeof(Ipv4Addr);
      destIpAddr.ipv4Addr = pseudoHeader->srcAddr;

      //Convert the port number to host byte order
      destPort = ntohs(udpHeader->srcPort);

      //Send NBNS response
      nbnsSendResponse(interface, &destIpAddr, destPort, message->id);
   }
}


/**
 * @brief Send NBNS response message
 * @param[in] interface Underlying network interface
 * @param[in] destIpAddr Destination IP address
 * @param[in] destPort destination port
 * @param[in] id 16-bit identifier to be used when sending NBNS query
 **/

error_t nbnsSendResponse(NetInterface *interface,
   const IpAddr *destIpAddr, uint16_t destPort, uint16_t id)
{
   error_t error;
   uint_t i;
   size_t length;
   size_t offset;
   NetBuffer *buffer;
   NbnsHeader *message;
   NbnsAddrEntry *addrEntry;
   DnsResourceRecord *record;
   Ipv4AddrEntry *entry;
   NetTxAncillary ancillary;

   //Initialize status code
   error = NO_ERROR;

   //Allocate a memory buffer to hold the NBNS response message
   buffer = udpAllocBuffer(DNS_MESSAGE_MAX_SIZE, &offset);
   //Failed to allocate buffer?
   if(buffer == NULL)
      return ERROR_OUT_OF_MEMORY;

   //Point to the NBNS header
   message = netBufferAt(buffer, offset);

   //Take the identifier from the query message
   message->id = id;

   //Format NBNS response header
   message->qr = 1;
   message->opcode = DNS_OPCODE_QUERY;
   message->aa = 1;
   message->tc = 0;
   message->rd = 1;
   message->ra = 1;
   message->z = 0;
   message->b = 0;
   message->rcode = DNS_RCODE_NOERROR;
   message->qdcount = 0;
   message->ancount = 0;
   message->nscount = 0;
   message->arcount = 0;

   //NBNS response message length
   length = sizeof(DnsHeader);

   //The NBNS response contains 1 answer resource record
   for(i = 0; i < IPV4_ADDR_LIST_SIZE && message->ancount == 0; i++)
   {
      //Point to the current entry
      entry = &interface->ipv4Context.addrList[i];

      //Check the state of the address
      if(entry->state == IPV4_ADDR_STATE_VALID)
      {
         //Check whether the address belongs to the same subnet as the source
         //address of the query
         if(ipv4IsOnSubnet(entry, destIpAddr->ipv4Addr))
         {
            //Encode the host name using the NBNS name notation
            length += nbnsEncodeName(interface->hostname,
               (uint8_t *) message + length);

            //Point to the corresponding resource record
            record = DNS_GET_RESOURCE_RECORD(message, length);

            //Fill in resource record
            record->rtype = HTONS(DNS_RR_TYPE_NB);
            record->rclass = HTONS(DNS_RR_CLASS_IN);
            record->ttl = HTONL(NBNS_DEFAULT_RESOURCE_RECORD_TTL);
            record->rdlength = HTONS(sizeof(NbnsAddrEntry));

            //Point to the address entry array
            addrEntry = (NbnsAddrEntry *) record->rdata;

            //Fill in address entry
            addrEntry->flags = HTONS(NBNS_G_UNIQUE | NBNS_ONT_BNODE);
            addrEntry->addr = entry->addr;

            //Update the length of the NBNS response message
            length += sizeof(DnsResourceRecord) + sizeof(NbnsAddrEntry);

            //Number of resource records in the answer section
            message->ancount++;
         }
      }
   }

   //Valid NBNS response?
   if(message->ancount > 0)
   {
      //The ANCOUNT field specifies the number of resource records in the
      //answer section
      message->ancount = htons(message->ancount);

      //Adjust the length of the multi-part buffer
      netBufferSetLength(buffer, offset + length);

      //Debug message
      TRACE_INFO("Sending NBNS message (%" PRIuSIZE " bytes)...\r\n", length);
      //Dump message
      dnsDumpMessage((DnsHeader *) message, length);

      //Additional options can be passed to the stack along with the packet
      ancillary = NET_DEFAULT_TX_ANCILLARY;

      //This flag tells the stack that the destination is on a locally attached
      //network and not to perform a lookup of the routing table
      ancillary.dontRoute = TRUE;

      //A response packet is always sent to the source UDP port and source IP
      //address of the request packet
      error = udpSendBuffer(interface, NULL, NBNS_PORT, destIpAddr, destPort,
         buffer, offset, &ancillary);
   }

   //Free previously allocated memory
   netBufferFree(buffer);

   //Return status code
   return error;
}

#endif
