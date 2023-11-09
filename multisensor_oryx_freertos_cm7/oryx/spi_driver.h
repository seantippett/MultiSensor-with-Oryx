/**
 * @file spi_driver.h
 * @brief SPI driver
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _SPI_DRIVER_H
#define _SPI_DRIVER_H

//Dependencies
#include "core/net.h"

//SPI driver
extern const SpiDriver spiDriver;

//SPI related functions
error_t spiInit(void);
error_t spiSetMode(uint_t mode);
error_t spiSetBitrate(uint_t bitrate);
void spiAssertCs(void);
void spiDeassertCs(void);
uint8_t spiTransfer(uint8_t data);

#endif
