#ifndef __PHY_KSZ9563__H
#define __PHY_KSZ9563__H

void checkRGMIIStatus(void);

#define GLOBAL_OPP_3__SW_RESET				0x01

#define GLOBAL_OPP_PME__PME_EN				0x02
#define GLOBAL_OPP_PME__PME_ACTIVE_HIGH		0x01

#define GLOBAL_OPP_PME__PME_ACTIVE_HIGH		0x01


#define SIZE_OF_GLOBAL_OPP_REG		0x24
#pragma pack(1)
struct STR_GLOBAL_OPPERATION_REGISTERS{
	uint8_t		CHIP_ID[3];		// 0x0000, 0x0001, 0x0002,
	uint8_t		GLOBAL_OPP_3;	// 0x0003
	uint8_t	    dummy_4_5[2];	// 0x0004, 0x0005
	uint8_t	    PME	;			// 0x0006
	uint8_t		dummy_7_E[8];	// 0x0007 - 0x000E
	uint8_t		SKU_ID;		    // 0x000F,
	uint32_t	int_status;		// 0x0010 - 0x0013
	uint32_t	int_mask;		// 0x0014 - 0x0017
	uint32_t	int_status_port; // 0x0018 - 0x001B
	uint32_t    int_mask_port;	// 0x001C- 0x001F
};
#define SIZE_OF_GLOBAL_IO_REG	0x130
struct STR_GLOBAL_IO_REGISTERS{
	uint8_t		serialIOCtrl;	// 0x0100
	uint8_t		dummy_1_3[3];	// 0x0101 - 0x0103
	uint32_t	IBMControl;		// 0x0104 - 0x0107
	uint8_t		dummy_8_C[5];	// 0x0108- 0x010C
	uint8_t		IODriveStrength;	// 0x010D
	uint8_t		dummy_E_F[2];	//0x010E 0x010F
	uint32_t	IBMStatus;		// 0x0110 - 0x0113
	uint8_t		dummy_14_1F[12];   //0x0114 0x011F
	uint32_t	ledOverride;	// 0x0120 - 0x0123
	uint32_t	ledOutput;		// 0x0124 - 0x0127
	uint32_t	ledSource;		// 0x0128 - 0x012C
};
struct STR_GLOBAL_PHY_CONTROL_STATUS_REGISTERS{
	uint8_t		dummy0;				// 0x0200
	uint8_t		powerDownCtrl_0;	// 0x0201
	uint8_t		dummy_2_F[14];		// 0x0202 - 0x020F
	uint32_t	ledStrapIn;			// 0x0210 - 0x0213
};

struct STR_GLOBAL_SWITCH_CONTROL_REGISTERS{
	uint8_t		switchOpp;			// 0x0300
	uint8_t		dummy_1;			// 0x0301
	uint8_t		switchMAC_Addr[6];	// 0x0302 - 0x0307
	uint16_t	switchMaxTxLen;	// 0x0308 - 0x309
	uint16_t	switchISPTPID;	// 0x30A - 0x30B
	uint16_t	dummy_C_D;
	uint16_t	AVB_CreditShaper;		// 0x030E-0x30F
	uint8_t		switchLookupCtrl;		// 0x0310
	uint8_t		switchLookupEngine[3];	// 0x0311 0x0313
	uint8_t		addrLookupInt;			// 0x0314
	uint8_t		addrLookupMask;			// 0x0315
	uint16_t	addrLookupTableEntry[3];	// 0x0316 - 0x031B
	uint8_t		dummy_1A_1F[6];
	uint32_t	unicastCtrlReg;			// 0x0320 - 0x0323
	uint32_t	multicastCtrlReg;			// 0x0324 - 0x0327
	uint32_t	VLAN_IDCtrlReg;			// 0x0328 - 0x032B
	uint8_t		dummy_2C_2F[4];			// 0x032C - 0x032F
	uint8_t		switchMACCtrl[6];			// 0x0330 - 0x0335
	uint8_t		swithcMIBCtrl;			//0x0336
	uint8_t		dummy_37;
	uint8_t		priorityMap[4];			//0x0338 - 0x033B
	uint8_t		dummy_3C_3D[2];			//0x033C - 0x033D
	uint8_t		IPDiffServPriorityEN;	// 0x033E
	uint8_t		dummy_3F;
	uint8_t		IPDiffServPriorityMAP[32];	// 0x0340 - 0x035F
	uint8_t		dummy_60[16];
	uint8_t		portMirror_SnoopingCtrl;	// 0x0370
	uint8_t		dummy_71_77[7];
	uint8_t		WREDDiffServColorMap;	// 0x0378
	uint8_t		dummy_79_7B[3];
	uint8_t		PTPEventMessagePriority;	// 0x037C
	uint8_t		PTPNonEventMessagePriority;	// 0x037D
	uint8_t		dummy_7E_8F[18];			// 0x037E-0x038F
	uint32_t	queueMgmtCtrl;				// 0x0390-0x393


};


struct STR_PORT_OPERATION_REGISTERS {
	uint8_t		operationCtrl;		//000
	uint8_t		defaultTag1;		//001
	uint8_t		dummy_2_3[2];
	uint8_t		avbSRClass1Tag0;	//004
	uint8_t		avbSRClass1Tag1;	//005
	uint8_t		avbSRClass2Tag0;	//006
	uint8_t		avbSRClass2Tag1;	//007
	uint16_t	avbSRClass1Type;	//008 -009
	uint16_t	avbSRClass2Type;	//00A -00B
	uint8_t		dummy_C_F[4];
	uint8_t		PME_WoLEvent;		//013
	uint8_t		dummy_14_16[3];
	uint8_t		PME_WoLEnable;		//017
	uint8_t		dummy_18_1A[3];
	uint8_t		portIntStatus;		//01B
	uint8_t		dummy_1C_1E[3];
	uint8_t		portIntMask;		//01F
	uint8_t		opControl0;			//0x020
	uint8_t		opControl1;			//0x021
	uint8_t		dummy_22_2F[14];	//0x022 - 0x02F
	uint8_t		status;				//0x030

};

struct STR_PORT_ETHERNET_PHY_REGISTERS{
	uint16_t	basicControl;		//0x100-0x101
	uint16_t	basicStatus;		//0x102 - 103
	uint16_t	phyID[2];			//0x104 - 107
	uint16_t	phyAutoNegAdvertizement; // 0x108 - 0x109
	uint16_t	phyAutoNegLinkPartnerAbility;	// 0x10A 0x10B
	uint16_t	phyAutoNegExpansionStatus;		// 0x10C 0x10D
	uint16_t	phyAutoNegNextPage;				// 0x10E 0x10F
	uint16_t	phyAutoNegLinkPartnerNextPageAbility;	// 0x110 - 0x111
	uint16_t	phy1000Base_tControl;			// 0x112 0x113
	uint16_t	phy1000Base_tStatus;			// 0x114 0x115
	uint8_t		dummy_16_19[4];					// 0x116 0119
	uint16_t	mmdSetupReg;					// 0x11A 0x11B
	uint16_t	mmdDataReg;					// 0x11C 0x11D
	uint16_t	extendedStatus;				// 0x11E 0x11F
	uint8_t		dummy_20_21[2];					// 0x116 0119
	uint16_t	remoteLoopback;				// 0x122 0x123
	uint16_t	linkMD;				// 0x124 125
	uint16_t	digitalPMA_PCS_status;	//126 127
	uint8_t		dummy_28_29[2];					// 0x128 0129.
	uint16_t	RXERR_Count;				// 12A 12B
	uint8_t		dummy_2C_35[10];					// 0x12C 0129.
	uint16_t	intCtrl_Status;				// 136 137
	uint16_t	phyAutoMDI_MDIX;				// 138 139
	uint8_t		dummy_3A_3D[4];					// 0x12C 0129.
	uint16_t	phyControlReg;				//13E 13F
};

struct STR_PORT_RGMII_CONTROL_REGISTERS{
	uint8_t		xmiiPortControl0;	// 0x300
	uint8_t		xmiiPortControl1;	// 0x301
	uint8_t		dummy_02;			// 0x302
	uint8_t		xmiiPortControl3;	// 0x303

};


struct KSZ9563_Registers{

	struct STR_GLOBAL_OPPERATION_REGISTERS globalOppReg;
	struct STR_GLOBAL_IO_REGISTERS	globalIOReg;
	struct STR_GLOBAL_PHY_CONTROL_STATUS_REGISTERS globalPhyOppStatus;
	struct STR_GLOBAL_SWITCH_CONTROL_REGISTERS globalSwitchCtrl;
	struct STR_PORT_OPERATION_REGISTERS portOppReg[3];
	struct STR_PORT_ETHERNET_PHY_REGISTERS portEthernetPhyReg[2];
	struct STR_PORT_RGMII_CONTROL_REGISTERS	portRGMIIControlReg;


};




#define KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL						0x0100
#define KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS						0x0102
#define KSZ_9563_ADDR_PORT__PHY_ID_0								0x0104
#define KSZ_9563_ADDR_PORT__PHY_ID_1								0x0106
#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT				0x0108
#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_LINK_PARTNER_ABILITY		0x010A

#define KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL					0x0112
#define KSZ_9563_ADDR_PORT__PHY_1000BASE_T_STATUS					0x0114


#define KSZ9563_REG__BASIC_CONTROL_PHY_RESET_BIT					((uint16_t)0x8000)
#define KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_0		((uint16_t)0x2000)	//	Bit 6 and 13 work as a pair.
#define KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_1		((uint16_t)0x0040)
#define KSZ9563_REG__BASIC_CONTROL_PHY_AUTO_NEG_ENABLE		((uint16_t)0x1000)
#define KSZ9563_REG__BASIC_CONTROL_PHY_RESET_AUTONEG_BIT	((uint16_t)0x0200)

#define KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_FD_Capeable_BIT		((uint16_t)0x0200)
#define KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_HD_Capeable_BIT		((uint16_t)0x0100)

#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_FD		((uint16_t)0x00100)
#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_HD		((uint16_t)0x00080)
#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_FD			((uint16_t)0x00040)
#define KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_HD			((uint16_t)0x00020)

#define KSZ_9563_ADDR_PORT__AUTO_MDIX_CONTROL	0x0138
#define KSZ9563_REG__AUTO_MDIX_CONTROL_SWAP_OFF_BIT	0x0020		// when SET AutoMDIX is OFF.
#define KSZ9563_REG__AUTO_MDIX_CONTROL_MDI_BIT		0x0040		// when SET we're in MDI mode.  MDIX is when clear.
#define KSZ_9563__COMMAND_READ		0b00001100000
#define KSZ_9563__COMMAND_WRITE	    0b00001000000

#define KSZ_9563_ADDR__GLOBAL_CHIP_ID				0x0000
#define KSZ_9563_ADDR__GLOBAL_INT_STATUS			0x0010
#define KSZ_9563_ADDR__GLOBAL_INT_MASK				0x0014
#define KSZ_9563_ADDR__GLOBAL_INT_STATUS_PORT		0x0018
#define KSZ_9563_ADDR__GLOBAL_INT_MASK_PORT			0x001C




#define KSZ_9563_ADDR__GLOBAL_IO_CONTROL_REG		0x0100
#define KSZ_9563_ADDR__GLOBAL_IO_SERIAL_IO_CONTROL_REG	0x0100
#define KSZ_9563_ADDR__GLOBAL_IO_IBM_CONTROL_REG	0x0104
#define KSZ_9563_ADDR__GLOBAL_IO_IO_DRIVE_STRENGTH	0x010D
#define KSZ_9563_ADDR__GLOBAL_IO_IBM_STATUS			0x0110
#define KSZ_9563_ADDR__GLOBAL_IO_LED_OVERRIDE		0x0120
#define KSZ_9563_ADDR__GLOBAL_IO_LED_OUTPUT			0x0124
#define KSZ_9563_ADDR__GLOBAL_IO_LED_SOURCE			0x0128




#define KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_REG	0x0200
#define KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_LED_STRAP_IN	0x0210


#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG	0x0300
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_SWITCH_MAX_TX_LEN	0x0308
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_SWITCH_ISPTPID		0x030A
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_AVB_CREDIT_SHAPER	0x030E
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_UNICAST_CTRL   0x0320
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_MULTICAST_CTRL   0x0324
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_VLAN_ID_CTRL   0x0328
#define KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_QUEUE_MGMT_CTRL   0x0328






#define KSZ_9563_ADDR__GLOBAL_SWITCH_LOOK_UP_REG	0x0400
#define KSZ_9563_ADDR__GLOBAL_SWITCH_PTP_REG		0x0500

#define KSZ_9563_ADDR_PORT_SELECT__1				0x1000
#define KSZ_9563_ADDR_PORT_SELECT__2				0x2000
#define KSZ_9563_ADDR_PORT_SELECT__3				0x3000

#define KSZ_9563_ADDR_PORT__OPERATION					0x0000
#define KSZ_9563_ADDR_PORT__OPERATION_AVB_SR_CLASS_1_TYPE				0x0008
#define KSZ_9563_ADDR_PORT__OPERATION_AVB_SR_CLASS_2_TYPE				0x000A




#define KSZ_9563_ADDR_PORT__PHY						0x0100



#define KSZ_9563_ADDR_PORT__RGMII					0x0300
#define KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_0	0x0300
#define KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_1	0x0301
#define KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_3	0x0303


#define KSZ_9563_ADDR_PORT__MAC						0x0400
#define KSZ_9563_ADDR_PORT__MIB_COUNTERS			0x0500

int32_t getLinkSideStatus(uint32_t side);
void setPhyLedsOnOff(uint32_t enable);
uint32_t getPhyLedState(void);
#endif
