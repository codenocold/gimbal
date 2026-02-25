#ifndef __UTIL_H__
#define __UTIL_H__

#include "main.h"

#define REG32(addr) (*(volatile uint32_t *) (uint32_t) (addr))

uint16_t crc16(uint16_t crc, uint8_t *buf, int size);
uint32_t crc32(uint32_t crc, uint8_t *buf, int size);

void     uint16_to_data(uint16_t val, uint8_t *data);
uint16_t data_to_uint16(uint8_t *data);

#endif
