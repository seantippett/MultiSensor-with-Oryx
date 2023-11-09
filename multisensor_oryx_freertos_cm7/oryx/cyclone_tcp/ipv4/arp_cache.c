/**
 * @file arp_cache.c
 * @brief ARP cache management
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
 * @section Description
 *
 * Address Resolution Protocol is used to determine the hardware address of
 * a specific host when only its IPv4 address is known. Refer to RFC 826
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

//Switch to the appropriate trace level
#define TRACE_LEVEL ARP_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "ipv4/arp_cache.h"
#include "ipv4/ipv4_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (IPV4_SUPPORT == ENABLED && ETH_SUPPORT == ENABLED)


/**
 * @brief Update ARP cache entry state
 * @param[in] entry Pointer to a ARP cache entry
 * @param[in] newState New state to switch to
 **/

void arpChangeState(ArpCacheEntry *entry, ArpState newState)
{
#if defined(ARP_CHANGE_STATE_HOOK)
   ARP_CHANGE_STATE_HOOK(entry, newState);
#endif

   //Save current time
   entry->timestamp = osGetSystemTime();
   //Switch to the new state
   entry->state = newState;
}


/**
 * @brief Create a new entry in the ARP cache
 * @param[in] interface Underlying network interface
 * @return Pointer to the newly created entry
 **/

ArpCacheEntry *arpCreateEntry(NetInterface *interface)
{
   uint_t i;
   systime_t time;
   ArpCacheEntry *entry;
   ArpCacheEntry *oldestEntry;

   //Get current time
   time = osGetSystemTime();

   //Keep track of the oldest entry
   oldestEntry = NULL;

   //Loop through ARP cache entries
   for(i = 0; i < ARP_CACHE_SIZE; i++)
   {
      //Point to the current entry
      entry = &interface->arpCache[i];

      //Check the state of the ARP entry
      if(entry->state == ARP_STATE_NONE)
      {
         //Initialize ARP entry
         osMemset(entry, 0, sizeof(ArpCacheEntry));

         //Return a pointer to the ARP entry
         return entry;
      }
      else if(entry->state == ARP_STATE_PERMANENT)
      {
         //Static ARP entries are never updated
      }
      else
      {
         //Keep track of the oldest entry in the table
         if(oldestEntry == NULL)
         {
            oldestEntry = entry;
         }
         else if(entry->state == ARP_STATE_STALE &&
            oldestEntry->state != ARP_STATE_STALE)
         {
            oldestEntry = entry;
         }
         else if(entry->state != ARP_STATE_STALE &&
            oldestEntry->state == ARP_STATE_STALE)
         {
         }
         else if((time - entry->timestamp) > (time - oldestEntry->timestamp))
         {
            oldestEntry = entry;
         }
         else
         {
         }
      }
   }

   //Any entry available in the ARP cache?
   if(oldestEntry != NULL)
   {
      //Drop any pending packets
      arpFlushQueuedPackets(interface, oldestEntry);
      //The oldest entry is removed whenever the table runs out of space
      arpChangeState(oldestEntry, ARP_STATE_NONE);
      //Initialize ARP entry
      osMemset(oldestEntry, 0, sizeof(ArpCacheEntry));
   }

   //Return a pointer to the ARP entry
   return oldestEntry;
}


/**
 * @brief Search the ARP cache for a given IPv4 address
 * @param[in] interface Underlying network interface
 * @param[in] ipAddr IPv4 address
 * @return A pointer to the matching ARP entry is returned. NULL is returned
 *   if the specified IPv4 address could not be found in ARP cache
 **/

ArpCacheEntry *arpFindEntry(NetInterface *interface, Ipv4Addr ipAddr)
{
   uint_t i;
   ArpCacheEntry *entry;

   //Loop through ARP cache entries
   for(i = 0; i < ARP_CACHE_SIZE; i++)
   {
      //Point to the current entry
      entry = &interface->arpCache[i];

      //Check whether the entry is currently in use
      if(entry->state != ARP_STATE_NONE)
      {
         //Current entry matches the specified address?
         if(entry->ipAddr == ipAddr)
         {
            return entry;
         }
      }
   }

   //No matching entry in ARP cache
   return NULL;
}


/**
 * @brief Flush ARP cache
 * @param[in] interface Underlying network interface
 **/

void arpFlushCache(NetInterface *interface)
{
   uint_t i;
   ArpCacheEntry *entry;

   //Loop through ARP cache entries
   for(i = 0; i < ARP_CACHE_SIZE; i++)
   {
      //Point to the current entry
      entry = &interface->arpCache[i];

      //Check the state of the ARP entry
      if(entry->state == ARP_STATE_PERMANENT)
      {
         //Static ARP entries are never updated
      }
      else
      {
         //Drop packets that are waiting for address resolution
         arpFlushQueuedPackets(interface, entry);

         //Delete ARP entry
         arpChangeState(entry, ARP_STATE_NONE);
      }
   }
}


/**
 * @brief Send packets that are waiting for address resolution
 * @param[in] interface Underlying network interface
 * @param[in] entry Pointer to a ARP cache entry
 **/

void arpSendQueuedPackets(NetInterface *interface, ArpCacheEntry *entry)
{
   uint_t i;
   size_t length;
   ArpQueueItem *item;

   //Check the state of the ARP entry
   if(entry->state == ARP_STATE_INCOMPLETE)
   {
      //Loop through the queued packets
      for(i = 0; i < entry->queueSize; i++)
      {
         //Point to the current queue item
         item = &entry->queue[i];

         //Retrieve the length of the IPv4 packet
         length = netBufferGetLength(item->buffer) - item->offset;
         //Update IP statistics
         ipv4UpdateOutStats(interface, entry->ipAddr, length);

         //Send the IPv4 packet
         ethSendFrame(interface, &entry->macAddr, ETH_TYPE_IPV4, item->buffer,
            item->offset, &item->ancillary);

         //Release memory buffer
         netBufferFree(item->buffer);
      }
   }

   //The queue is now empty
   entry->queueSize = 0;
}


/**
 * @brief Flush packet queue
 * @param[in] interface Underlying network interface
 * @param[in] entry Pointer to a ARP cache entry
 **/

void arpFlushQueuedPackets(NetInterface *interface, ArpCacheEntry *entry)
{
   uint_t i;

   //Check the state of the ARP entry
   if(entry->state == ARP_STATE_INCOMPLETE)
   {
      //Drop packets that are waiting for address resolution
      for(i = 0; i < entry->queueSize; i++)
      {
         //Release memory buffer
         netBufferFree(entry->queue[i].buffer);
      }
   }

   //The queue is now empty
   entry->queueSize = 0;
}

#endif
