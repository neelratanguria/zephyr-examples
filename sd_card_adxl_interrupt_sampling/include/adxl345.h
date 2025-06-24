#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define ADXL345_DEVID_REG       0x00
#define ADXL345_POWER_CTL_REG   0x2D
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_BW_RATE_REG     0x2C

// #define ADXL345_DEVID           0xE5
#define ADXL345_DEVID           0xCB
#define ADXL345_MEASURE         0x08
#define ADXL345_RANGE_2G        0x00
#define ADXL345_RATE_100HZ      0x0A  // Output Data Rate = 100 Hz
#define ADXL345_RATE_3200HZ     0x0F  // Output Data Rate =  3200 Hz


#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_READ_MULTI (1 << 7 | 1 << 6)  // bit7=Read, bit6=Multi-byte

// struct spi_dt_spec spispec;
int check_spi_ready(void);
int release_spi(void);

int adxl345_spi_read_reg(uint8_t reg_addr, uint8_t *val);
int adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z);
int adxl345_write_reg(uint8_t reg, uint8_t val);
int adxl345_read_reg(uint8_t reg, uint8_t *val);
int adxl345_init(void);
