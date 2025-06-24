#include "adxl345.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adxl345, LOG_LEVEL_DBG);



struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(adxl345), SPIOP, 0);

int check_spi_ready(void)
{
    if (!spi_is_ready_dt(&spispec)) {
        LOG_ERR("SPI device not ready!");
        return -ENODEV;
    }
    return 0;
}

int release_spi(void)
{
    spi_release_dt(&spispec);
    LOG_INF("SPI device released.");
    return 0;
}


int adxl345_spi_read_reg(uint8_t reg_addr, uint8_t *val)
{
    uint8_t tx_buf[2] = { reg_addr | 0x80, 0x00 };  // Set read bit
    uint8_t rx_buf[2] = { 0 };

    struct spi_buf tx = {
        .buf = tx_buf,
        .len = 2
    };
    struct spi_buf rx = {
        .buf = rx_buf,
        .len = 2
    };

    struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1
    };

    int ret = spi_transceive_dt(&spispec, &tx_set, &rx_set);
    if (ret == 0) {
        *val = rx_buf[1];  // second byte is data
    }
    return ret;
}

int adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t tx_buf[7] = { ADXL345_REG_DATAX0 | ADXL345_READ_MULTI };  // 1-byte cmd + 6 dummy
    uint8_t rx_buf[7] = { 0 };

    struct spi_buf tx = {
        .buf = tx_buf,
        .len = 7
    };
    struct spi_buf rx = {
        .buf = rx_buf,
        .len = 7
    };

    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(&spispec, &tx_set, &rx_set);
    if (ret != 0) {
        return ret;
    }

    // Combine little-endian bytes
    *x = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    *y = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    *z = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);

    return 0;
}

int adxl345_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[] = { reg & 0x3F, val };  // Write command
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    return spi_write_dt(&spispec, &tx_set);
}

int adxl345_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx_buf[2] = { reg | 0x80, 0x00 };  // Read command
    uint8_t rx_buf[2] = { 0 };

    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(&spispec, &tx_set, &rx_set);
    if (ret == 0) *val = rx_buf[1];
    return ret;
}

int adxl345_init(void)
{
    if (!spi_is_ready_dt(&spispec)) {
        LOG_ERR("SPI device not ready!\n");
        return -ENODEV;
    }

    uint8_t devid;
    int ret = adxl345_read_reg(ADXL345_DEVID_REG, &devid);
    if (ret != 0 || devid != ADXL345_DEVID) {
        LOG_ERR("ADXL345 not detected! Read 0x%02X\n", devid);
        return -EIO;
    }

    LOG_INF("ADXL345 detected. DEVID: 0x%02X\n", devid);

    // Set BW_RATE to 100 Hz
    if (adxl345_write_reg(ADXL345_BW_RATE_REG, ADXL345_RATE_3200HZ)) {
        LOG_ERR("Failed to set BW_RATE\n");
        return -EIO;
    }

    // Set data format to ±2g, right-justified, full-res disabled
    if (adxl345_write_reg(ADXL345_DATA_FORMAT_REG, ADXL345_RANGE_2G)) {
        LOG_ERR("Failed to set data format\n");
        return -EIO;
    }

    // Enable measurement mode
    if (adxl345_write_reg(ADXL345_POWER_CTL_REG, ADXL345_MEASURE)) {
        LOG_ERR("Failed to enable measurement\n");
        return -EIO;
    }

    LOG_INF("ADXL345 initialization complete.\n");
    return 0;
}