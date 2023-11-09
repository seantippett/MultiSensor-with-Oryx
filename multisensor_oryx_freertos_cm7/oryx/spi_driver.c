/**
 * @file spi_driver.c
 * @brief SPI driver
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

//Dependencies
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "core/net.h"
#include "spi_driver.h"
#include "debug.h"

//SPI bit rate
#define SPI_BITRATE 10000000


/**
 * @brief SPI driver
 **/

const SpiDriver spiDriver =
{
   spiInit,
   spiSetMode,
   spiSetBitrate,
   spiAssertCs,
   spiDeassertCs,
   spiTransfer
};


/**
 * @brief SPI initialization
 * @return Error code
 **/

error_t spiInit(void)
{
   uint32_t freq;
   gpio_pin_config_t pinConfig;
   lpspi_master_config_t lpspiMasterConfig;

   //Debug message
   TRACE_INFO("Initializing SPI driver...\r\n");

   //Configure GPIO_AD_29 as GPIO9_IO28
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_29_GPIO9_IO28, 0);

   //Set GPIO_AD_29 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_29_GPIO9_IO28,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUE(0) |
      IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(0));

   //Configure CS pin
   pinConfig.direction = kGPIO_DigitalOutput;
   pinConfig.outputLogic = 1;
   pinConfig.interruptMode = kGPIO_NoIntmode;
   GPIO_PinInit(GPIO9, 28, &pinConfig);

   //Deassert CS
   GPIO_PinWrite(GPIO9, 28, 1);

   //Configure GPIO_AD_28 as LPSPI1_SCK
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_28_LPSPI1_SCK, 0);

   //Set GPIO_AD_28 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_28_LPSPI1_SCK,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUE(0) |
      IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(0));

   //Configure GPIO_AD_30 as LPSPI1_SDO
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_30_LPSPI1_SOUT, 0);

   //Set GPIO_AD_30 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_30_LPSPI1_SOUT,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUE(0) |
      IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(0));

   //Configure GPIO_AD_31 as LPSPI1_SDI
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_31_LPSPI1_SIN, 0);

   //Set GPIO_AD_31 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_31_LPSPI1_SIN,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
      IOMUXC_SW_PAD_CTL_PAD_PUE(0) |
      IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(0));

   //Get LPSPI1 root clock frequency
   freq = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpspi1);

   //Get default LPSPI configuration
   LPSPI_MasterGetDefaultConfig(&lpspiMasterConfig);
   lpspiMasterConfig.bitsPerFrame = 8;
   lpspiMasterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
   lpspiMasterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
   lpspiMasterConfig.direction = kLPSPI_MsbFirst;
   lpspiMasterConfig.baudRate = SPI_BITRATE;
   lpspiMasterConfig.pcsToSckDelayInNanoSec = 1000000000U / SPI_BITRATE * 2U;
   lpspiMasterConfig.lastSckToPcsDelayInNanoSec = 1000000000U / SPI_BITRATE * 2U;
   lpspiMasterConfig.betweenTransferDelayInNanoSec = 1000000000U / SPI_BITRATE * 2U;
   lpspiMasterConfig.whichPcs = kLPSPI_Pcs0;

   //Configure LPSPI module
   LPSPI_MasterInit(LPSPI1, &lpspiMasterConfig, freq);

   //Successful processing
   return NO_ERROR;
}


/**
 * @brief Set SPI mode
 * @param mode SPI mode (0, 1, 2 or 3)
 **/

error_t spiSetMode(uint_t mode)
{
   //Not implemented
   return ERROR_NOT_IMPLEMENTED;
}


/**
 * @brief Set SPI bitrate
 * @param bitrate Bitrate value
 **/

error_t spiSetBitrate(uint_t bitrate)
{
   //Not implemented
   return ERROR_NOT_IMPLEMENTED;
}


/**
 * @brief Assert CS
 **/

void spiAssertCs(void)
{
   //Assert CS
   GPIO_PinWrite(GPIO9, 28, 0);
   //CS setup time
   usleep(1);
}


/**
 * @brief Deassert CS
 **/

void spiDeassertCs(void)
{
   //CS hold time
   usleep(1);
   //Deassert CS
   GPIO_PinWrite(GPIO9, 28, 1);
   //CS disable time
   usleep(1);
}


/**
 * @brief Transfer a single byte
 * @param[in] data The data to be written
 * @return The data received from the slave device
 **/

uint8_t spiTransfer(uint8_t data)
{
#if 0
   uint8_t temp;
   lpspi_transfer_t lpspiTransfer;

   //Configure SPI transfer
   lpspiTransfer.txData = &data;
   lpspiTransfer.rxData = &temp;
   lpspiTransfer.dataSize = sizeof(uint8_t);
   lpspiTransfer.configFlags = 0;

   //Perform SPI transfer
   LPSPI_MasterTransferBlocking(LPSPI1, &lpspiTransfer);

   //Return the received character
   return temp;
#else
   uint8_t temp;

   //Enable module
   LPSPI1->CR |= LPSPI_CR_MEN_MASK;
   //Reset TX and RX FIFOs
   LPSPI1->CR |= LPSPI_CR_RTF_MASK | LPSPI_CR_RRF_MASK;

   //Set FIFO watermark
   LPSPI1->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0);

   //Clear flags
   LPSPI1->SR = LPSPI_SR_DMF_MASK | LPSPI_SR_REF_MASK | LPSPI_SR_TEF_MASK |
      LPSPI_SR_TCF_MASK | LPSPI_SR_FCF_MASK | LPSPI_SR_WCF_MASK;

   //Wait for the TX FIFO to be available for writing
   while((LPSPI1->SR & LPSPI_SR_TDF_MASK) == 0)
   {
   }

   //Send data
   LPSPI1->TDR = data;

   //Wait for the RX FIFO to be available for reading
   while((LPSPI1->SR & LPSPI_SR_RDF_MASK) == 0)
   {
   }

   //Receive data
   temp =  (uint8_t) LPSPI1->RDR;

   //Wait for the transfer to complete
   while((LPSPI1->SR & LPSPI_SR_TCF_MASK) == 0)
   {
   }

   //Return the received character
   return temp;
#endif
}
