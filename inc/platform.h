#ifndef __TIMESTAMP_H
#define __TIMESTAMP_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

// Assume Vdd = 3.3V
#define TSENSOR_ADC2T100(n) ((1799356-948*n) >> 9)

uint64_t timestamp_get(void);

// UART
size_t uart_strlen(const char * str);
void uart_puts(const char *str);
void uart_put(const char c);
void uart_int32(int n);

// CRC
void crc32_init(void);
#define crc32_hash8(dat) (*(((uint8_t*)(&(CRC->DR)))) = dat)
#define crc32_hash16(dat) (*(((uint16_t*)(&(CRC->DR)))) = dat)
#define crc32_hash32(dat) (*(((uint32_t*)(&(CRC->DR)))) = dat)
#define crc32_result() (CRC->DR ^ 0xFFFFFFFF)

// ADC
uint16_t adc_read(uint32_t chan);
uint16_t adc_read_temperature(void);

void hw_init(void);

#endif // __TIMESTAMP_H
