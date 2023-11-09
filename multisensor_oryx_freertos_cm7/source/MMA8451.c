



#include "MMA8451.h"
#include "app_shared.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freeRTOS.h"
#include "fsl_iomuxc.h"

int I2C_HAL_rx(LPI2C_Type *base, lpi2c_master_transfer_t *transfer);
int I2C_HAL_tx(LPI2C_Type *base, lpi2c_master_transfer_t *transfer);
int I2C_HAL_init(LPI2C_Type *base);
//struct i2c_master_packet packet;
extern struct i2c_master_module i2c_master_instance;

volatile uint8_t MMA_write_buffer[2];
volatile uint8_t MMA_read_buffer;

uint16_t AD_fifo_head, AD_fifo_tail, AD_fifo_tail_tcpip, eng_packet_fifo_tail, eng_packet_counter, MMA8451_initialized;


/* Get frequency of lpi2c clock */
#define LPI2C_MMA8451_CLOCK_FREQUENCY (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))


#define LPI2C_MMA8451_BASEADDR 			LPI2C6
#define LPI2C_MMA8451_IRQ 			  LPI2C6_IRQn
#define LPI2C_MMA8451_IRQHandler 	  LPI2C6_IRQHandler


#define LPI2C_MMA8451_BAUDRATE               150000U	// Changed to 200kHz.  I have trouble running this above 215kHz.  MMA8451 spec'd up to 400kHz.    T.S. 2022 07 24
														// with "High drive strength", This can run up to 350kHz without problems, however the wavefomrs look a bit on the edge.
#define LPI2C_MMA8451_MAX_DATA_LENGTH            250U


//volatile uint8_t g_masterTxIndex         = 0U;
//volatile uint8_t g_masterRxIndex         = 0U;
//uint8_t g_master_TxLen					 = 0U;
//uint8_t g_master_RxLen					 = 0U;
//volatile bool g_masterReadBegin          = false;

uint8_t MMA8451_default_register_values[] =
{
  
    //*****  setup primary device parameters  *****
  MMA8451_ADDR_CTRL_REG2, MMA8451_SOFT_RESET,    // reset the chip for good measure
    
  MMA8451_ADDR_CTRL_REG1, 0x00 ,    // ensure we're in standby mode and set rate

//  MMA8451_ADDR_CTRL_REG2, 0 /*MMA8451_AUTO_SLEEP_EN*/,    // not enable autosleep

//  MMA8451_ADDR_ASLP_COUNT_REG, 1,    // set autosleep time (320ms / bit)
  
  MMA8451_ADDR_F_SETUP, (MMA8451_FMODE_FIFO_CIRCULAR_MODE + 8),  // set to cirular fill mode + interupt level.

  MMA8451_ADDR_CTRL_REG3, (MMA8451_WAKE_TRANS + MMA8451_WAKE_LNDPRT),    //set interupt to neg polarity
 
  MMA8451_ADDR_CTRL_REG4, (0x40 /*+ 0x20 + 0x10 + 0x08 + 0x04*/),    // enable fifo int, transient interupt, not rotation, pulse, freefall
 
  MMA8451_ADDR_CTRL_REG5, (0x40 /*+ 0x20 + 0x10 + 0x08 + 0x04*/),    // map fifo + transient interupt to interupt 1, all others to int 2  
  
  MMA8451_ADDR_XYZ_DATA_CFG, MMA8451_RANGE_8G,   // Set full scale range to 8G  
     
  
  ////*****  setup transient detect  *****
  //MMA8451_TRANSIENT_CFG_REG, 0x1E,        //all axeses + latch
  //
  //MMA8451_TRANSIENT_THS_REG, 2,           //set threshold  (0.063g / bit)
  //
  //MMA8451_TRANSIENT_CNT_REG, 1,           //set threshold debounce count (# of samples)
 //
  //MMA8451_HP_FILTER_REG, 3,           //set HPF cutoff to 2Hz (3 at 800Hz sample rate, 0 @ 50Hz)
  
  
  ////*****  setup rotation detect  *****
  //MMA8451_PL_CONFIG_REG, 0x40,        //enable orientation and clear counter when condition no longer valid
 //
  //MMA8451_PL_COUNT_REG, 0x50,        //set to max time (.319s @ 800Hz sample rate)
 //
  //MMA8451_PL_BF_ZCOMP_REG, 0x47,        //these are the default( 75 degree front/back trip, 43 degree Z-lock) 
 //
  //MMA8451_PL_THS_REG, 0x81,       //45 degree threshold angle with 4 degree hysteresis 


  ////*****  setup pulse detect  *****  
  //MMA8451_PULSE_CFG_REG, 0x7F,        //enable single/double taps on all axes
  //
  //MMA8451_PULSE_THSX_REG, 32,        //x thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
//
  //MMA8451_PULSE_THSY_REG, 32,        //y thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
//
  //MMA8451_PULSE_THSZ_REG, 32,        //z thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
//
  //MMA8451_PULSE_TMLT_REG, 0x30,        //30ms time limit at 800Hz odr
 //
  //MMA8451_PULSE_LTCY_REG, 0xA0,        //200ms (at 800Hz odr) between taps min
 //
  //MMA8451_PULSE_WIND_REG, 0xFF,        //318ms (max value) between taps max

  
  //*****  setup freefall detect  *****
    //not implemented.  But really... should we?  I mean... if the device is falling off the fence, we might want to know?
    // Don't you think?  Maybe it's like Newton's apple... Some kid is going to re-discover gravity when this falls on his
  // head.  Shouldn't we flag that so we can catch it on camera?  Or... if this is being sold in the USA... maybe we
  // should turn off the camera so there's no evidence of this event and thus we don't get our asses sued off!
   
  //*****  Finalize initialization  *****
  MMA8451_ADDR_CTRL_REG1, (MMA8451_ACTIVE_BIT + MMA8451_DATA_RATE_800Hz)          // turn Active mode back on.

};




/*
 * INT8U MMA8451_init( void )
 * 
 *		Returns 0 on success.
 *				1 otherwise.
   */
lpi2c_rtos_handle_t  	lpi2C_MMA8451_Handle;
lpi2c_master_transfer_t MMA8451masterXfer;
lpi2c_master_config_t 	lpi2C_MMA8451_Config = {0};

uint8_t MMA8451_init( void ){
	uint8_t  i, status;
	volatile uint8_t g_master_buff[LPI2C_MMA8451_MAX_DATA_LENGTH] = {0};
	// TODO, I think the above is only required to be volatile for the I2C transfer.  And yet it was declared volatile throughout.  I've typecast'd to stop the warnings... but there's better ways to deal with this.
//	uint8_t *buffPtr;

//	NVIC_SetPriority(LPI2C_MMA8451_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);//configMAX_SYSCALL_INTERRUPT_PRIORITY);//configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);


	// POLLING SETUP.
#if(0)
	LPI2C_MasterGetDefaultConfig(&lpi2C_MMA8451_Config);
	lpi2C_MMA8451_Config.baudRate_Hz = LPI2C_MMA8451_BAUDRATE;
	LPI2C_MasterInit(LPI2C_MMA8451_BASEADDR, &lpi2C_MMA8451_Config, LPI2C_MMA8451_CLOCK_FREQUENCY);
#endif
	I2C_HAL_init(LPI2C_MMA8451_BASEADDR);



    memset((uint8_t *)g_master_buff, 0, sizeof(g_master_buff));
    memset(&MMA8451masterXfer, 0, sizeof(MMA8451masterXfer));
    MMA8451masterXfer.slaveAddress   = MMA8451_DEV_ADD;
    MMA8451masterXfer.direction      = kLPI2C_Read;
    MMA8451masterXfer.subaddress     = MMA8451_ADDR_WHO_AM_I;
    MMA8451masterXfer.subaddressSize = 1;
    MMA8451masterXfer.data           = (uint8_t *)g_master_buff;
    MMA8451masterXfer.dataSize       = 1;
    MMA8451masterXfer.flags          = kLPI2C_TransferDefaultFlag;


#if(0)
    status = LPI2C_MasterTransferBlocking(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
#endif
    status = I2C_HAL_rx(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);


    if (status != kStatus_Success)    {
    	LPI2C_MasterReset(LPI2C_MMA8451_BASEADDR);
        return 1;
    }

    if(g_master_buff[0] != MMA8451_WHO_AM_I_RESPONSE){
    	return 1;    }
    vTaskDelay(10 / portTICK_PERIOD_MS );


    // write
    MMA8451masterXfer.direction      = kLPI2C_Write;
    for(i = 0; i < sizeof(MMA8451_default_register_values); i+=2){
        vTaskDelay(2 / portTICK_PERIOD_MS );
        MMA8451masterXfer.subaddress     = MMA8451_default_register_values[i  ];
        MMA8451masterXfer.data           = &(MMA8451_default_register_values[i+1]);

//        status = LPI2C_MasterTransferBlocking(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
        status = I2C_HAL_tx(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
    }
    status = 0;
    memset((uint8_t *)g_master_buff, 0, sizeof(g_master_buff));
    status = 0;
    // read back.
    MMA8451masterXfer.direction      = kLPI2C_Read;
    for(i = 0; i < sizeof(MMA8451_default_register_values); i+=2){
        vTaskDelay(2 / portTICK_PERIOD_MS );
        MMA8451masterXfer.subaddress     = MMA8451_default_register_values[i  ];
        MMA8451masterXfer.data           = (uint8_t *)&(g_master_buff[i]);

//        status |= LPI2C_MasterTransferBlocking(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
        status = I2C_HAL_rx(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
    }
    if(status != 0){
    	return 1;	// fail.

    }
//    // compare
    for(i = 4; i < sizeof(MMA8451_default_register_values); i+=2){
        if(MMA8451_default_register_values[i+1] != g_master_buff[i]){
        	return 1;		// FAIL!
        }
    }

	return 0;
}


// samples is returned as x,y,z,x,y,z,x,y,z.... format.
// samplesCount is returned as number of 3 sample samples.  I.e. x,y,z is ONE sample.  Max is 32 sets of 3 samples.
// samples is returned as signed 16 bit numbers.
uint8_t MMA8451_read_FIFO(int16_t *samples, uint16_t *samplesCount){
	uint8_t temp, i,j, status;
	volatile uint8_t g_master_buff[LPI2C_MMA8451_MAX_DATA_LENGTH] = {0};

	union U_data{
		struct S_14bitSampleSet{
			int16_t xData	:14;
			int16_t yData	:14;
			int16_t zData	:14;
		}samples[32];
		uint8_t rawData[192];
	}data;

    MMA8451masterXfer.slaveAddress   = MMA8451_DEV_ADD;
    MMA8451masterXfer.direction      = kLPI2C_Read;
    MMA8451masterXfer.subaddress     = MMA8451_ADDR_F_STATUS;
    MMA8451masterXfer.subaddressSize = 1;
    MMA8451masterXfer.data           = (uint8_t *)&(g_master_buff[0]);
    MMA8451masterXfer.dataSize       = 1;
    MMA8451masterXfer.flags          = kLPI2C_TransferDefaultFlag;


//    status |= LPI2C_MasterTransferBlocking(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
    status = I2C_HAL_rx(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);

    if(status != 0){
    	*samplesCount = 0;
    	return status;	// fail
    }

    i = g_master_buff[0] & MMA8451_FIFO_FCNT_MASK;
    if(i > 32){
    	i = 0;
    	return 1; // fail.
    }
    if(i > 0){
        MMA8451masterXfer.subaddress     = MMA8451_ADDR_X_AXIS_DATA;
        MMA8451masterXfer.data           = &(data.rawData[0]); // &(g_master_buff[0]);
        MMA8451masterXfer.direction      = kLPI2C_Read;
        MMA8451masterXfer.dataSize       = (i * 2 * 3);

//        status |= LPI2C_MasterTransferBlocking(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);
        status = I2C_HAL_rx(LPI2C_MMA8451_BASEADDR, &MMA8451masterXfer);

    }
    // i = # of samples PER AXIS!
    for(j = 0; j < (i * 2 * 3)-1; j+=2){// byte swap for endienness
    	temp = data.rawData[j];
    	data.rawData[j] = data.rawData[j+1];
		data.rawData[j+1] = temp;
    }

    for(temp = 0; temp <i; temp++){
    	samples[temp * 3] = data.samples[temp].xData;
    	samples[temp * 3 + 1] = data.samples[temp].yData;
    	samples[temp * 3 + 2] = data.samples[temp].zData;
    }
    if(status != 0){
    	*samplesCount = 0;
    	return status;	// fail
    }

    *samplesCount = i;

    return 0;
}

int I2C_HAL_init(LPI2C_Type *base)
{
	// this is based on the b2b polling example.

	int i;
	lpi2c_master_config_t masterConfig;
	// FLUSH the port.
	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );

	gpio_pin_config_t gpioOutput_config = {
	  .direction = kGPIO_DigitalOutput,
	  .outputLogic = 0U,
	  .interruptMode = kGPIO_NoIntmode
	};

	LPI2C_MasterGetDefaultConfig(&masterConfig);

	if(base == LPI2C6){


		GPIO_PinInit(GPIO12, 7U, &gpioOutput_config);	//SCL
		IOMUXC_SetPinMux(
			IOMUXC_GPIO_LPSR_07_GPIO12_IO07,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
			0U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */

		for(i = 0; i <16; i++){
			GPIO_PinWrite(GPIO12, (07U), 0U);		// SCL
			vTaskDelay(1 / portTICK_PERIOD_MS );
			GPIO_PinWrite(GPIO12, (07U), 1U);		// SCL
			vTaskDelay(1 / portTICK_PERIOD_MS );
		}
		IOMUXC_SetPinMux(
			IOMUXC_GPIO_LPSR_07_LPI2C6_SCL,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
			1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */
		IOMUXC_SetPinConfig(						//T.S. high drive strength 2022 07 25
			IOMUXC_GPIO_LPSR_07_LPI2C6_SCL,         /* GPIO_LPSR_06 PAD functional properties : */
			0x2EU);                                 /* Slew Rate Field: Slow Slew Rate */
	    /* Change the default baudrate configuration */
	    masterConfig.baudRate_Hz = LPI2C_MMA8451_BAUDRATE;

	}else if(base == LPI2C5){

		GPIO_PinInit(GPIO12, 5U, &gpioOutput_config);	//SCL
		IOMUXC_SetPinMux(
			IOMUXC_GPIO_LPSR_05_GPIO12_IO05,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
			0U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */

		for(i = 0; i <16; i++){
			GPIO_PinWrite(GPIO12, (05U), 0U);		// SCL
			vTaskDelay(1 / portTICK_PERIOD_MS );
			GPIO_PinWrite(GPIO12, (05U), 1U);		// SCL
			vTaskDelay(1 / portTICK_PERIOD_MS );
		}
		IOMUXC_SetPinMux(
			IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
			1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */
		IOMUXC_SetPinConfig(						//T.S. high drive strength 2022 07 25
			IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_06 PAD functional properties : */
			0x2EU);                                 /* Slew Rate Field: Slow Slew Rate */
	    /* Change the default baudrate configuration */
	    masterConfig.baudRate_Hz = LPI2C_MMA8451_BAUDRATE;
	}






    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(base, &masterConfig, LPI2C_MMA8451_CLOCK_FREQUENCY);
    LPI2C_MasterStop(base);

	xSemaphoreGive(xI2CSemaphore);
	return 0;

}

int I2C_HAL_tx(LPI2C_Type *base, lpi2c_master_transfer_t *transfer)
{
	size_t txCount = 0;
	int reVal;
    /* Send master blocking data to slave */
	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );
    if (kStatus_Success == LPI2C_MasterStart(base, transfer->slaveAddress, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(base) & kLPI2C_MasterNackDetectFlag)
        {
        	xSemaphoreGive(xI2CSemaphore);
            return kStatus_LPI2C_Nak;
        }

        /* subAddress = 0x01, data = g_master_txBuff - write to slave.
          start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
        reVal = LPI2C_MasterSend(base, &transfer->subaddress, transfer->subaddressSize);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(base);
            }
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }

        reVal = LPI2C_MasterSend(base, transfer->data, transfer->dataSize);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(base);
            }
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }

        reVal = LPI2C_MasterStop(base);
        if (reVal != kStatus_Success)
        {
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }
    }
	xSemaphoreGive(xI2CSemaphore);
    return -1;
}

int I2C_HAL_rx(LPI2C_Type *base, lpi2c_master_transfer_t *transfer)
{
	int i;
	size_t txCount = 0;
	int reVal;
	xSemaphoreTake(xI2CSemaphore,  ( TickType_t ) portMAX_DELAY );

    if (kStatus_Success == LPI2C_MasterStart(base, transfer->slaveAddress, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(base) & kLPI2C_MasterNackDetectFlag)
        {
        	xSemaphoreGive(xI2CSemaphore);
            return kStatus_LPI2C_Nak;
        }

        reVal = LPI2C_MasterSend(base, &transfer->subaddress, transfer->subaddressSize);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(base);
            }
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }

        reVal = LPI2C_MasterRepeatedStart(base, transfer->slaveAddress, kLPI2C_Read);
        if (reVal != kStatus_Success)
        {
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }

        reVal = LPI2C_MasterReceive(base, transfer->data, transfer->dataSize);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(base);
            }
        	xSemaphoreGive(xI2CSemaphore);
            return -1;
        }
        reVal = 0;
        for(i = 0; i < 5; i++){
			reVal = LPI2C_MasterStop(base);
			if((reVal == kStatus_Success) || (reVal == kStatus_LPI2C_FifoError)){
				//break;
				xSemaphoreGive(xI2CSemaphore);
				return 0;
			}
			vTaskDelay(1 / portTICK_PERIOD_MS );
        }


        if (reVal != kStatus_Success)
		{
        	xSemaphoreGive(xI2CSemaphore);
			return -1;
		}
    	xSemaphoreGive(xI2CSemaphore);
        return 0;
    }
	xSemaphoreGive(xI2CSemaphore);
    return -1;
}
