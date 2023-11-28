/*
 * app_board.c
 *
 *  Created on: Jun. 28, 2022
 *      Author: tsnider
 */


#include <string.h>
#include <math.h>
#include <simpleFlashSupport.h>

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"
#include "fsl_lpspi.h"
#include "fsl_lpi2c_freertos.h"
#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"

#include "core/net.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"

#include "app_board.h"
#include "app_pir.h"
#include "app_config.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_freertos.h"
#include "fsl_cache.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "app_config.h"
#include "phy_ksz9563.h"


//#define EXAMPLE_LPSPI_MASTER_BASEADDR         (LPSPI1)
//#define EXAMPLE_LPSPI_MASTER_IRQN             (LPSPI1_IRQn)
#define LPSPI_PCS_FOR_INIT     (kLPSPI_Pcs0)
#define LPSPI_PCS__KSZ9536	   (kLPSPI_MasterPcs0)

#define LPSPI_CLK_FREQ (CLOCK_GetFreqFromObs(CCM_OBS_LPSPI1_CLK_ROOT))

#define EXAMPLE_LPSPI_DEALY_COUNT 0xFFFFFU
#define TRANSFER_SIZE     256U     /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 5000000U /*! Transfer baudrate - 5000k */



__DATA(SRAM_DTC_cm7) uint8_t masterRxData[TRANSFER_SIZE] = {0U};
__DATA(SRAM_DTC_cm7) uint8_t masterTxData[TRANSFER_SIZE] = {0U};
volatile uint32_t g_systickCounter  = 20U;




lpi2c_rtos_handle_t  	lpi2C_PHY_Handle;
lpi2c_master_transfer_t masterPHYXfer;
lpi2c_master_config_t 	lpi2C_PHY_Config = {0};

lpspi_rtos_handle_t 	lpspi_PHY_Handle;
lpspi_master_config_t 		lpspi_PHY_Config = {0};
lpspi_transfer_t		masterPHYXfer_SPI;


/* Get frequency of lpi2c clock */



#define LPI2C_PHY_CLOCK_FREQUENCY (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))
#define LPI2C_PHY_BASEADDR LPI2C1
#define LPI2C_PHY_IRQn		LPI2C1_IRQn
#define LPI2C_PHY_BAUDRATE	50000


#define PHY_LPSPI_MASTER_BASEADDR         (LPSPI1)
#define PHY_LPSPI_MASTER_IRQN             (LPSPI1_IRQn)
#define PHY_LPSPI_MASTER_PCS_FOR_INIT     (kLPSPI_Pcs0)
#define PHY_LPSPI_MASTER_PCS_FOR_TRANSFER (kLPSPI_MasterPcs0)
#define PHY_MASTER_CLK_FREQ (CLOCK_GetFreqFromObs(CCM_OBS_LPSPI1_CLK_ROOT))
#define PHY_LPSPI_DEALY_COUNT 0xFFFFFU
#define LPSPI_PHY_BAUDRATE  5000000





#pragma pack(1)
	union U_commandAddr{
		struct S_commandAddr{
				uint16_t command	:11;
				uint16_t addr		:16;
				uint8_t	 dontCare	:5;
			}message;
		uint8_t raw8[4];
		uint32_t raw32;
	} commandAddr;
#pragma pack()


#pragma pack(1)
struct KSZ9563_Registers KSZ9563_registers;

union U_byteSwapping16{
	uint16_t raw16;
	uint8_t  raw8[2];
};
union U_byteSwapping32{
	uint32_t raw32;
	uint16_t raw16[2];
	uint8_t  raw8[4];
};


void initKSZ9563Phy(void){
	uint16_t status;

	memset(masterRxData, 0, sizeof(masterRxData));
	memset(masterTxData, 0, sizeof(masterTxData));

	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );

	NVIC_SetPriority(LPI2C_PHY_IRQn, 5);
	LPI2C_MasterGetDefaultConfig(&lpi2C_PHY_Config);
	lpi2C_PHY_Config.baudRate_Hz = LPI2C_PHY_BAUDRATE;

	status = LPI2C_RTOS_Init(&lpi2C_PHY_Handle, LPI2C_PHY_BASEADDR, &lpi2C_PHY_Config, LPI2C_PHY_CLOCK_FREQUENCY);
	LPI2C_MasterCheckAndClearError(LPI2C_PHY_BASEADDR, status);

	xSemaphoreGive(xI2CSemaphore);

	masterPHYXfer.slaveAddress = 0x5F;
	masterPHYXfer.direction      = kLPI2C_Read;
	masterPHYXfer.subaddress     = masterTxData[0];
	masterPHYXfer.subaddressSize = 2;
	masterPHYXfer.data           = masterRxData;
	masterPHYXfer.dataSize       = sizeof(KSZ9563_registers.globalOppReg);;
	masterPHYXfer.flags          = kLPI2C_TransferDefaultFlag;



}

void initKSZ9563Phy_SPI(void){
//	uint16_t status;
    uint32_t srcClock_Hz;

	memset(masterRxData, 0, sizeof(masterRxData));
	memset(masterTxData, 0, sizeof(masterTxData));

	NVIC_SetPriority(LPSPI1_IRQn, 5);

	LPSPI_MasterGetDefaultConfig(&lpspi_PHY_Config);
	lpspi_PHY_Config.baudRate = LPSPI_PHY_BAUDRATE;
	lpspi_PHY_Config.whichPcs = PHY_LPSPI_MASTER_PCS_FOR_INIT;

	masterPHYXfer_SPI.txData   = masterTxData;
	masterPHYXfer_SPI.rxData   = NULL;
	masterPHYXfer_SPI.dataSize = TRANSFER_SIZE;
	masterPHYXfer_SPI.configFlags =	PHY_LPSPI_MASTER_PCS_FOR_INIT | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
	srcClock_Hz = LPSPI_CLK_FREQ;
//    LPSPI_MasterInit(PHY_LPSPI_MASTER_BASEADDR, &lpspi_PHY_Config, srcClock_Hz);

	LPSPI_RTOS_Init(&lpspi_PHY_Handle, PHY_LPSPI_MASTER_BASEADDR, &lpspi_PHY_Config, srcClock_Hz);



}
void getKSZ9563_I2C(uint16_t addr, uint8_t *data, uint16_t size){
//	uint8_t status;

#if(1)
	union U_byteSwapping32 byteSwap;

	byteSwap.raw32 = addr << 5;
	byteSwap.raw32 |= 0x60000000;	// read command.
	memset(masterRxData, 0, sizeof(masterRxData));
	masterTxData[0] = byteSwap.raw8[3];
	masterTxData[1] = byteSwap.raw8[2];
	masterTxData[2] = byteSwap.raw8[1];
	masterTxData[3] = byteSwap.raw8[0];
	masterTxData[4] = 0;
	masterTxData[5] = 0;
	masterTxData[6] = 0;
	masterTxData[7] = 0;

	/*Start master transfer, transfer data to slave.*/
	masterPHYXfer_SPI.txData   = masterTxData;
	masterPHYXfer_SPI.rxData   = masterRxData;
	masterPHYXfer_SPI.dataSize = size + 4;
	masterPHYXfer_SPI.configFlags =	LPSPI_PCS__KSZ9536 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
//    LPSPI_MasterTransferBlocking(PHY_LPSPI_MASTER_BASEADDR, &masterPHYXfer_SPI);
	LPSPI_RTOS_Transfer(&lpspi_PHY_Handle, &masterPHYXfer_SPI);


#else

	masterTxData[1] = (uint8_t) (addr / 256);
	masterTxData[0] = (uint8_t) (addr & 0xFF);
	masterPHYXfer.direction      = kLPI2C_Read;
	masterPHYXfer.subaddress     = addr;
	masterPHYXfer.subaddressSize = 2;
	masterPHYXfer.data           = masterRxData;
	masterPHYXfer.dataSize       = size;
	masterPHYXfer.flags          = kLPI2C_TransferDefaultFlag;

	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );
    status = LPI2C_RTOS_Transfer(&lpi2C_PHY_Handle, &masterPHYXfer);
	xSemaphoreGive(xI2CSemaphore);
#endif

    memcpy(data, &(masterRxData[4]), size);

}

void setKSZ9563_I2C(uint16_t addr, uint8_t *data, uint16_t size){

	uint32_t i;

#if(1)
	union U_byteSwapping32 byteSwap;

	byteSwap.raw32 = addr << 5;
	byteSwap.raw32 |= 0x40000000;	// Write command.

	masterTxData[0] = byteSwap.raw8[3];
	masterTxData[1] = byteSwap.raw8[2];
	masterTxData[2] = byteSwap.raw8[1];
	masterTxData[3] = byteSwap.raw8[0];

	for(i = 0; i < size; i++)
	{
		masterTxData[4 + i] = data[i];
	}
	/*Start master transfer, transfer data to slave.*/
	masterPHYXfer_SPI.txData   = masterTxData;
	masterPHYXfer_SPI.rxData   = masterRxData;
	masterPHYXfer_SPI.dataSize = size + 4;
	masterPHYXfer_SPI.configFlags =	LPSPI_PCS__KSZ9536 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    LPSPI_MasterTransferBlocking(PHY_LPSPI_MASTER_BASEADDR, &masterPHYXfer_SPI);
//	LPI2C_RTOS_Transfer(&lpspi_PHY_Handle, &masterPHYXfer_SPI);				 we don't use the freeRtos transfer because we init the device before scheduler.

#else
	masterTxData[1] = (uint8_t) (addr / 256);
	masterTxData[0] = (uint8_t) (addr & 0xFF);
	memcpy(masterRxData, data, size);


	masterPHYXfer.direction      = kLPI2C_Write;
	masterPHYXfer.subaddress     = addr;
	masterPHYXfer.subaddressSize = 2;
	masterPHYXfer.data           = masterRxData;
	masterPHYXfer.dataSize       = size;
	masterPHYXfer.flags          = kLPI2C_TransferDefaultFlag;



	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );
		status = LPI2C_RTOS_Transfer(&lpi2C_PHY_Handle, &masterPHYXfer);
	xSemaphoreGive(xI2CSemaphore);
#endif

}

void setKSZ9563Reg(uint16_t addr, uint8_t *data, uint16_t size){

	uint8_t swappedData[4];
	if(size == 4){
		swappedData[3] = data[0];
		swappedData[2] = data[1];
		swappedData[1] = data[2];
		swappedData[0] = data[3];
	}else if(size == 2){
		swappedData[1] = data[0];
		swappedData[0] = data[1];
	}else if(size == 1){
		swappedData[0] = data[0];
	}

	setKSZ9563_I2C(addr, swappedData, size);


}
void getKSZ9563Reg(uint16_t addr, uint8_t *data, uint16_t size){
	uint8_t swappedData[4];
	getKSZ9563_I2C(addr, swappedData, size);
	if(size == 4){
		data[0] = swappedData[3];
		data[1] = swappedData[2];
		data[2] = swappedData[1];
		data[3] = swappedData[0];
	}else if(size == 2){
		data[0] = swappedData[1];
		data[1] = swappedData[0];
	}else if(size == 1){
		data[0] = swappedData[0];
	}
}

RAMFUNCTION_SECTION_CODE(void resetBoard(void))
{
	vTaskSuspendAll();
	while(1){ // trigger watchdog this way. the other ways above may work but this for sure works

	}
    //__disable_irq();
    //NVIC_SystemReset();

}

int32_t getLinkSideStatus(uint32_t side)
{
	if(side == 0){
		getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS),
							(uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicStatus),
							sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicStatus));

		if(KSZ9563_registers.portEthernetPhyReg[0].basicStatus & 0x04){
			// link is up.
			return 1;
		}else{
			return 0;
		}

	}else if(side == 1){
	    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS),
	    					(uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicStatus),
							sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicStatus));

		if(KSZ9563_registers.portEthernetPhyReg[1].basicStatus & 0x04){
			// link is up.
			return 1;
		}else{
			return 0;
		}
	}else{
		return -1;
	}
}



int setupKSZ9563_Phy(void){

    // HW REV 2
	GPIO_WritePinOutput(GPIO9, 8, 0);			// I have a board here that isn't shitting around with the 10mS mark.  SO don't be short.  EVER.
    vTaskDelay(100 / portTICK_PERIOD_MS );		//
	GPIO_WritePinOutput(GPIO9, 8, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS );		//


	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_AD_28_LPSPI1_SCK,
	  1U);
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_AD_28_LPSPI1_SCK,
	  0x00);
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_AD_29_LPSPI1_PCS0 ,
	  1U);
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_AD_29_LPSPI1_PCS0 ,
	  0x00);
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_AD_30_LPSPI1_SOUT ,
	  1U);
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_AD_30_LPSPI1_SOUT ,
	  0x00);
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_AD_31_LPSPI1_SIN ,
	  1U);
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_AD_31_LPSPI1_SIN ,
	  0x00);


#if(0)
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x03), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[2]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[2]));
    KSZ9563_registers.globalOppReg.CHIP_ID[2] = KSZ9563_registers.globalOppReg.CHIP_ID[2] | 0x02;	// global chip reset.
    setKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x03), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[2]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[2]));
#endif
    vTaskDelay(300 / portTICK_PERIOD_MS );		//  Rise time on the RESET_N line is 200mS.  Give it 50% safety margin.   T.S.

    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x00), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[0]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[0]));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x01), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[1]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[1]));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x02), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[2]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[2]));

    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x00), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[0]), 4);

    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID + 0x02), (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID[2]), sizeof(KSZ9563_registers.globalOppReg.CHIP_ID[2]));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));
    KSZ9563_registers.globalSwitchCtrl.switchOpp = KSZ9563_registers.globalSwitchCtrl.switchOpp | 0x02;	// global chip reset.
    setKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));


    vTaskDelay(1000 / portTICK_PERIOD_MS );		//  I'm only guessing we need this delay after reset.  T.S.
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));
    KSZ9563_registers.globalSwitchCtrl.switchOpp = KSZ9563_registers.globalSwitchCtrl.switchOpp | 0x01;	// global chip start.
    setKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG + 0x00), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp), sizeof(KSZ9563_registers.globalSwitchCtrl.switchOpp));

	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_0), (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0), 1);
	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_1), (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1), 1);
	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_3), (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3), 1);



	KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1 &=  0xFB;	// clear in band status register.
	KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1 |= 0x10;		// internal delay on RGMII clk..  This is super important for Rev2 cards.
	setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII_XMII_PORT_CTRL_1), (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1), 1);


    vTaskDelay(5 / portTICK_PERIOD_MS );		//

//    vTaskSuspendAll();	// pause the scheduler, so we don't task swap here.

    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_REG + 0x10), (uint8_t *) &(KSZ9563_registers.globalPhyOppStatus.ledStrapIn), sizeof(KSZ9563_registers.globalPhyOppStatus.ledStrapIn));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x00, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x01, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x03, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_ID_0), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyID[0]), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyID[0]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_ID_1), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyID[1]), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyID[1]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_ID_0), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyID[0]), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyID[0]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_ID_1), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyID[1]), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyID[1]));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_ID_0), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyID[0]), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyID[0]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_ID_1), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyID[1]), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyID[1]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_ID_0), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyID[0]), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyID[0]));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_ID_1), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyID[1]), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyID[1]));

//    xTaskResumeAll();	// pause the scheduler, so we don't task swap here.

    if(!((KSZ9563_registers.portEthernetPhyReg[0].phyID[0] == 0x22) &&
    	(KSZ9563_registers.portEthernetPhyReg[1].phyID[0] == 0x22) &&
    	(KSZ9563_registers.portEthernetPhyReg[0].phyID[1] == 0x1637) &&
    	(KSZ9563_registers.portEthernetPhyReg[1].phyID[1] == 0x1637))  ) {

    	return -1;
    }
#if(0)
// PHY reset.  // It's important to read back the BASIC Control register.  I have a board here that stays link off until you read that back.
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));
    KSZ9563_registers.portEthernetPhyReg[0].basicControl = KSZ9563_registers.portEthernetPhyReg[0].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_RESET_BIT;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
    KSZ9563_registers.portEthernetPhyReg[1].basicControl = KSZ9563_registers.portEthernetPhyReg[1].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_RESET_BIT;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
//
//    xTaskDelayUntil( &wakeTimer, 	100 / portTICK_PERIOD_MS );
    vTaskDelay(1000 / portTICK_PERIOD_MS );
#endif
//      getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicStatus));
//      getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicStatus));

// setup for 100Mbit.
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));
    KSZ9563_registers.portEthernetPhyReg[0].basicControl = KSZ9563_registers.portEthernetPhyReg[0].basicControl & ~KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_1;
    KSZ9563_registers.portEthernetPhyReg[0].basicControl = KSZ9563_registers.portEthernetPhyReg[0].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_0;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
    KSZ9563_registers.portEthernetPhyReg[1].basicControl = KSZ9563_registers.portEthernetPhyReg[1].basicControl & ~KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_1;
    KSZ9563_registers.portEthernetPhyReg[1].basicControl = KSZ9563_registers.portEthernetPhyReg[1].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_SPEED_SELECT_BIT_0;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));

// clear the gigabit advertizement.

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl));
    KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl = KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl & (~( KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_FD_Capeable_BIT | KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_HD_Capeable_BIT));
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl));


    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl));
    KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl = KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl & (~( KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_FD_Capeable_BIT | KSZ9563_REG__PHY_1000BaseT_CONTROL_1000BaseT_HD_Capeable_BIT));
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl));
    // PC is connected at this point.
    //  setup the remaining auto Neg advertizement

    vTaskDelay(1000 / portTICK_PERIOD_MS );		// this delay, and the one after auto neg. advertizement adjustments. was added.  I have a rev 2 card here that didn't link up without these delays.  They may be excessively long... didn't experiment with that.

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement));
    KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement = KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement  | (KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_FD \
 		   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_HD \
																																		  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_FD \
																																		  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_HD);
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement));
    KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement = KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement  | (KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_FD \
 		   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_100TX_HD \
																																		  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_FD \
																																		  | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT_10T_HD);
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement));

    vTaskDelay(1000 / portTICK_PERIOD_MS );		// this delay, and the one before auto neg. advertizement adjustments. was added.  I have a rev 2 card here that didn't link up without these delays.  They may be excessively long... didn't experiment with that.

    // set to operate on AUTO MDIX
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__AUTO_MDIX_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX));
    KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX = KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX & (~(KSZ9563_REG__AUTO_MDIX_CONTROL_SWAP_OFF_BIT));
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__AUTO_MDIX_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoMDI_MDIX));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__AUTO_MDIX_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX));
    KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX = KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX & (~(KSZ9563_REG__AUTO_MDIX_CONTROL_SWAP_OFF_BIT));
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__AUTO_MDIX_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoMDI_MDIX));

// now turn on auto Neg.

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));
    KSZ9563_registers.portEthernetPhyReg[0].basicControl = KSZ9563_registers.portEthernetPhyReg[0].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_AUTO_NEG_ENABLE;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
    KSZ9563_registers.portEthernetPhyReg[1].basicControl = KSZ9563_registers.portEthernetPhyReg[1].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_AUTO_NEG_ENABLE;
    setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));

#if(1)
    // restart Auto Neg. (not sure this is necessary... but seems like we should give it a go
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));
	KSZ9563_registers.portEthernetPhyReg[0].basicControl = (uint16_t)KSZ9563_registers.portEthernetPhyReg[0].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_RESET_AUTONEG_BIT;
	setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicControl));

	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
	KSZ9563_registers.portEthernetPhyReg[1].basicControl = (uint16_t)KSZ9563_registers.portEthernetPhyReg[1].basicControl | KSZ9563_REG__BASIC_CONTROL_PHY_RESET_AUTONEG_BIT;
	setKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicControl));
#endif
    vTaskDelay(500 / portTICK_PERIOD_MS );		//  I'm only guessing we need this delay after reset.  T.S.

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[0].basicStatus));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegAdvertizement));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_LINK_PARTNER_ABILITY), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegLinkPartnerAbility), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phyAutoNegLinkPartnerAbility));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tControl));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[0].phy1000Base_tStatus));

    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[1].basicStatus));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_ADDVERTIZEMENT), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegAdvertizement));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_AUTO_NEG_LINK_PARTNER_ABILITY), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegLinkPartnerAbility), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phyAutoNegLinkPartnerAbility));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_CONTROL), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tControl));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_1000BASE_T_STATUS), (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tStatus), sizeof(KSZ9563_registers.portEthernetPhyReg[1].phy1000Base_tStatus));



    if(configParam.visible_LEDs_Enable == TRUE){
    	// normal state
    	setPhyLedsOnOff(TRUE);
    }else{
    	// visible LED's are off.
    	setPhyLedsOnOff(FALSE);
    }


#if(0)
int i;
    for(i = 0; i < sizeof(struct STR_GLOBAL_OPPERATION_REGISTERS); i++){
    	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_CHIP_ID) + i, (uint8_t *) &(KSZ9563_registers.globalOppReg.CHIP_ID) +i, 1);
    }
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_INT_STATUS), (uint8_t *) &(KSZ9563_registers.globalOppReg.int_status), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_INT_MASK), (uint8_t *) &(KSZ9563_registers.globalOppReg.int_mask), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_INT_STATUS_PORT), (uint8_t *) &(KSZ9563_registers.globalOppReg.int_status_port), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_INT_MASK_PORT), (uint8_t *) &(KSZ9563_registers.globalOppReg.int_mask_port), 4);

    for(i = 0; i < sizeof(struct STR_GLOBAL_IO_REGISTERS); i++) {
    	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_CONTROL_REG) + i, (uint8_t *) &(KSZ9563_registers.globalIOReg.serialIOCtrl)+i, 1);
    }
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_SERIAL_IO_CONTROL_REG), (uint8_t *) &(KSZ9563_registers.globalIOReg.serialIOCtrl), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_IBM_CONTROL_REG), (uint8_t *) &(KSZ9563_registers.globalIOReg.IBMControl), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_IBM_STATUS), (uint8_t *) &(KSZ9563_registers.globalIOReg.IBMStatus), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OVERRIDE), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOverride), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OUTPUT), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOutput), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_SOURCE), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledSource), 4);

    for(i = 0; i < sizeof(struct STR_GLOBAL_PHY_CONTROL_STATUS_REGISTERS); i++) {
    	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_REG) + i, (uint8_t *) &(KSZ9563_registers.globalPhyOppStatus.dummy0)+i, 1);
    }
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_LED_STRAP_IN), (uint8_t *) &(KSZ9563_registers.globalPhyOppStatus.ledStrapIn), 4);



    for(i = 0; i < sizeof(struct STR_GLOBAL_SWITCH_CONTROL_REGISTERS); i++) {
    	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_REG) + i, (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchOpp)+i, 1);
    }
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_SWITCH_MAX_TX_LEN), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchMaxTxLen), 2);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_SWITCH_ISPTPID), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.switchISPTPID), 2);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_AVB_CREDIT_SHAPER), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.AVB_CreditShaper), 2);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_UNICAST_CTRL), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.unicastCtrlReg), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_MULTICAST_CTRL), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.multicastCtrlReg), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_VLAN_ID_CTRL), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.VLAN_IDCtrlReg), 4);
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_SWITCH_CONTROL_QUEUE_MGMT_CTRL), (uint8_t *) &(KSZ9563_registers.globalSwitchCtrl.queueMgmtCtrl), 4);

    for(i = 0; i < sizeof(struct STR_PORT_OPERATION_REGISTERS); i++) {
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__OPERATION) + i, (uint8_t *) &(KSZ9563_registers.portOppReg[0].operationCtrl)+i, 1);
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__OPERATION) + i, (uint8_t *) &(KSZ9563_registers.portOppReg[1].operationCtrl)+i, 1);
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__OPERATION) + i, (uint8_t *) &(KSZ9563_registers.portOppReg[2].operationCtrl)+i, 1);
    }

    		// these are all 16bit, so read 2 by 2.
    for(i = 0; i < sizeof(struct STR_PORT_ETHERNET_PHY_REGISTERS); i+=2) {
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__1 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL) + i, (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[0].basicControl)+i, 2);
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__2 | KSZ_9563_ADDR_PORT__PHY_BASIC_CONTROL) + i, (uint8_t *) &(KSZ9563_registers.portEthernetPhyReg[1].basicControl)+i, 2);
    }

    for(i = 0; i < sizeof(struct STR_PORT_RGMII_CONTROL_REGISTERS); i++) {
    	getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + i, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0)+i, 1);
    }
#endif
	return 0;
}

void checkRGMIIStatus(void){
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_PHY_CONTROL_REG + 0x10), (uint8_t *) &(KSZ9563_registers.globalPhyOppStatus.ledStrapIn), sizeof(KSZ9563_registers.globalPhyOppStatus.ledStrapIn));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x00, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl0));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x01, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl1));
    getKSZ9563Reg((KSZ_9563_ADDR_PORT_SELECT__3 | KSZ_9563_ADDR_PORT__RGMII) + 0x03, (uint8_t *) &(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3), sizeof(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3));

    if(KSZ9563_registers.portRGMIIControlReg.xmiiPortControl3 & 0x01){
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("RGMII: ONLINE \r\n");
		xSemaphoreGive( xPrintMutex );
    }else{
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("RGMII: xxxOFFLINExxx \r\n");
		xSemaphoreGive( xPrintMutex );

    }

}


uint32_t getPhyLedState(void){
	// we're just trusting our saved state of these.
	if(KSZ9563_registers.globalIOReg.ledOverride == 0){
		return TRUE;// leds enabled.
	}
	return FALSE; // leds disabled.
}

void setPhyLedsOnOff(uint32_t enable){
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OVERRIDE), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOverride), sizeof(KSZ9563_registers.globalIOReg.ledOverride));
    getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OUTPUT), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOutput), sizeof(KSZ9563_registers.globalIOReg.ledOutput));

    if(enable == TRUE){
       	// visible LED's are on.
        KSZ9563_registers.globalIOReg.ledOverride = 0x0000;
        KSZ9563_registers.globalIOReg.ledOutput = 0x0000;
    }else{
    	// visible LED's are off.
        KSZ9563_registers.globalIOReg.ledOverride = 0x000F;
        KSZ9563_registers.globalIOReg.ledOutput = 0x000F;
    }

	setKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OVERRIDE), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOverride), sizeof(KSZ9563_registers.globalIOReg.ledOverride));
	setKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OUTPUT), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOutput), sizeof(KSZ9563_registers.globalIOReg.ledOutput));

	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OVERRIDE), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOverride), sizeof(KSZ9563_registers.globalIOReg.ledOverride));
	getKSZ9563Reg((KSZ_9563_ADDR__GLOBAL_IO_LED_OUTPUT), (uint8_t *) &(KSZ9563_registers.globalIOReg.ledOutput), sizeof(KSZ9563_registers.globalIOReg.ledOutput));

}

