/**
 * @file stp_config.h
 * @brief CycloneSTP configuration file
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _STP_CONFIG_H
#define _STP_CONFIG_H

//Trace level for STP bridge
#define STP_TRACE_LEVEL TRACE_LEVEL_INFO
//Trace level for RSTP bridge
#define RSTP_TRACE_LEVEL TRACE_LEVEL_INFO

//STP bridge support
#define STP_SUPPORT DISABLED
//RSTP bridge support
#define RSTP_SUPPORT ENABLED

//BRIDGE MIB module support
#define BRIDGE_MIB_SUPPORT ENABLED
//Support for SET operations
#define BRIDGE_MIB_SET_SUPPORT ENABLED

//RSTP MIB module support
#define RSTP_MIB_SUPPORT ENABLED
//Support for SET operations
#define RSTP_MIB_SET_SUPPORT ENABLED


#endif
