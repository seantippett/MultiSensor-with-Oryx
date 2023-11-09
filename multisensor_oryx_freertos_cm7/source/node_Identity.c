/*
 * poc_nodes.c
 *
 *  Created on: Jun. 18, 2021
 *      Author: stippett
 */
#include <node_Identity.h>

#include "fsl_ocotp.h"
#include "fsl_debug_console.h"
#include "app_shared.h"
//#include "McuRTOS.h"



uint32_t uid[2];
#if(0)
static const uint32_t uid_node_table[] = {
		0x00000000, 0x00000000, /* default node 0 */
		0x82967DB4, 0x2F23280E, /* node 01 - TR20440482 */
		0x82967DB4, 0x122A280E, /* node 02 - TR20440007 */
		0x82967DB4, 0x3724280E, /* node 03 - TR20440440 */
		0x82967DB4, 0x2920280E, /* node 04 - TR20440303 */
		0x82967DB4, 0x2B18280E, /* node 05 - TR20440050 */
		0x82967DB4, 0x3A23280E, /* node 06 - TR20440376 */
		0x82967DB4, 0x3A2A280E, /* node 07 - TR20440101 */
		0x82967DB4, 0x111A180E, /* node 08 - TR21030700 */
		0x82967D6A, 0x3308080E, /* node 09 - TR21030869 */
		0x82967D6A, 0x300A080E, /* node 10 - TR21030785 */
		0x82967DB4, 0x2E23180E, /* node 11 - TR21030855 */
		0x82967D6A, 0x0F0C080E, /* node 12 - TR21030659 */
		0x82967DB4, 0x3B1B280E, /* node 13 - TR20440474 */
		0x82967DB4, 0x262D280E, /* node 14 - TR20440130 */
		0x82967DB4, 0x1125180E, /* node 15 - TR20440101 */
		0x00000000, 0x00000000, /* node 16 - sn# */
		0x00000000, 0x00000000, /* node 17 - sn# */
		0x00000000, 0x00000000, /* node 18 - sn# */
		0x00000000, 0x00000000, /* node 19 - sn# */
		0x00000000, 0x00000000, /* node 20 - sn# */
		0x00000000, 0x00000000, /* node 21 - sn# */
		0x00000000, 0x00000000, /* node 22 - sn# */
		0x00000000, 0x00000000, /* node 23 - sn# */
		0x00000000, 0x00000000, /* node 24 - sn# */
		0x00000000, 0x00000000, /* node 25 - sn# */
		0x00000000, 0x00000000, /* node 26 - sn# */
		0x00000000, 0x00000000, /* node 27 - sn# */
		0x00000000, 0x00000000, /* node 28 - sn# */
		0x00000000, 0x00000000, /* node 29 - sn# */
		0x00000000, 0x00000000, /* node 30 - sn# */
		0x00000000, 0x00000000, /* node 31 - sn# */
		0x82967DB4, 0x332D280E  /* node 32 - TR20440161 */
};
#else
static const uint32_t uid_node_table[] = {
		0x00000000, 0x00000000, /* default node 0 */
		0x828F2C4F, 0x2D23280E, /* node 01 - Tim's Card */
		0x828F2C4F, 0x2F0F380E, /* node 02 - TR20440007 */
		0x828F2C4F, 0x2813380E, /* node 03 - TR20440440 */
		0x828F2C4F, 0x1812380E, /* node 04 - TR20440303 */
		0x828F2C4F, 0x2112380E, /* node 05 - TR20440050 */
		0x828F2C4F, 0x1A09380E, /* node 06 - TR20440376 */
		0x828F2C4F, 0x240A380E, /* node 07 - TR20440101 */
		0x82924bfc, 0x0614080e, /* node 08 - TR21030700 */
		0x828F2C4F, 0x3A18280E, /* node 09 - TR21030869 */
		0x828F2C4F, 0x2916280E, /* node 10 - TR21030785 */
		0x828F2C4F, 0x2710380E, /* node 11 - TR21030855 */
		0x828F2C4F, 0x1813380e, /* node 12 - TR21030659 */
		0x828F2C4F, 0x240b380e, /* node 13 - TR20440474 */
		0x82924bfc, 0x2a15080e, /* node 14 - TR20440130 */
		0x828F2C4F, 0x3d18280e, /* node 15 - TR20440101 */
		0x828F2C4F, 0x100e380e, /* node 16 - sn# */
		0x828F2C4F, 0x260a380e, /* node 17 - sn# */
		0x828F2C4F, 0x0a10380e, /* node 18 - sn# */
		0x828F2C4F, 0x2918280e, /* node 19 - sn# */
		0x828F2C4F, 0x1C0C380E, /* node 20 - sn# */
		0x82924bfc, 0x2415080e, /* node 21 - sn# */
		0x82924bfc, 0x2314080e, /* node 22 - sn# */
		0x82924bfc, 0x2615080e, /* node 23 - sn# */
		0x82924bfc, 0x3a13080e, /* node 24 - sn# */
		0x82924bfc, 0x1015080e, /* node 25 - sn# */
		0x82924bfc, 0x3715080e, /* node 26 - sn# */
		0x82924bfc, 0x0915080e, /* node 27 - sn# */
		0x82924bfc, 0x0715080e, /* node 28 - sn# */
		0x82924bfc, 0x0815080e, /* node 29 - sn# */
		0x82924bfc, 0x2f15080e, /* node 30 - sn# */
		0x82924bfc, 0x1315080e, /* node 31 - sn# */
		0x82924bfc, 0x3015080e, /* node 32 -  */
		0x82924bfc, 0x3813080e, /* node 33 -  */
		0x82924bfc, 0x2115080e, /* node 34 -  */
		0x82924bfc, 0x1d15080e, /* node 35 -  */
		0x82924bfc, 0x1115080e, /* node 36 -  */
		0x82924bfc, 0x0e15080e, /* node 37 -  */
		0x82924bfc, 0x1815080e, /* node 38 -  */
		0x82924BFC, 0x2A12080E, /* node 39 -  */   // REV2 card.
		0x00000000, 0x00000000  /* node 40 -  */
};

#endif
status_t getUID(void){

	status_t status   = kStatus_Success;
	OCOTP_Init(OCOTP, 0U);

	status = OCOTP_ReadFuseShadowRegisterExt(OCOTP, OCOTP_ADDRESS(UID_FUSE_ADDRESS1), &uid[0], 1);
	status |= OCOTP_ReadFuseShadowRegisterExt(OCOTP, OCOTP_ADDRESS(UID_FUSE_ADDRESS2), &uid[1], 1);
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	PRINTF("i.MX RT1176 unique ID: 0x%08X 0x%08X\r\n", uid[0], uid[1]);
	xSemaphoreGive( xPrintMutex );

	return status;
}

uint32_t nodeNumLookup(){
	uint8_t node;
	for (node = 0; node <= NODE_TABLE_LENGTH; node++){
		if (uid[0] == uid_node_table[node*2]){
			if(uid[1] == uid_node_table[(node*2)+1]){
				return node;
			}
		}
	}
	return 0; // node not found in table so we default to node 0
}

uint32_t AppGetNodeNumber( void ){
	uint32_t nodeNum = 0;
	status_t status;
	status = getUID();
	if (status == kStatus_Success){
		nodeNum = nodeNumLookup();
	}

	return nodeNum;
}
