#pragma once
#include <stdint.h>
// Reflected CRC-16 CCITT calculation
#define CRC_INITIAL 0xFFFF
#define CRC_OK 0xF0B8

//void initCRC(void);
void CRCCalc(uint8_t bData, uint16_t *crc);
uint16_t CRCCalcRange(uint8_t *bDataStart, uint32_t items);

//uint16_t getCRC(void);
