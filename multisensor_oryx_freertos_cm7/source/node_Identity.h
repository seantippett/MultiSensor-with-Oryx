/*
 * poc_ips.h
 *
 *	Contains Unique ID -> IP and MAC address mapping, as well as the functions to retrieve them
 *
 *  Created on: Jun. 17, 2021
 *      Author: stippett
 */

#ifndef POC_IPS_H_
#define POC_IPS_H_

#include "fsl_common.h"

/*******************************************************************************
 * Config Definitions
 ******************************************************************************/
#define NODES_IN_TABLE		41

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UID_FUSE_ADDRESS1	0x900
#define UID_FUSE_ADDRESS2	0x910
#define NODE_TABLE_LENGTH	(2*NODES_IN_TABLE) + 2 // 2 entries per node + 2 entries for the default node 0
/*******************************************************************************
 * Functions
 ******************************************************************************/
#define OCOTP_ADDRESS(x) (((x - 0x800) >> 4)) // ((Address that is to be programmed/read) - 0x800) >> 4
uint32_t AppGetNodeNumber( void );
extern uint32_t uid[2];

/*******************************************************************************
 * Tasks
 ******************************************************************************/
status_t getUID(void);

#endif /* POC_IPS_H_ */
