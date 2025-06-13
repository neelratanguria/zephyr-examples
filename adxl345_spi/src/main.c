
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(Lesson5_Exercise1, LOG_LEVEL_INF);

/*
 * ADXL345 to nRF52833 SPI Pin Mapping
 *
 * | ADXL345 Pin | nRF52833 Pin     | Notes                  |
 * |-------------|------------------|------------------------|
 * | VCC         | 3.3V             | Use VDD from nRF52833  |
 * | GND         | GND              | Common ground          |
 * | CS          | P0.27            | Controlled by software |
 * | SCL (SCLK)  | P0.23            | SPI clock              |
 * | SDA (MOSI)  | P0.2             | Data from nRF to ADXL  |
 * | SDO (MISO)  | P0.26            | Data from ADXL to nRF  |
 * | INT1        | P0.15 (optional) | If using interrupts    |
 * | INT2        | -                | Not used               |
 */



#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_READ_MULTI (1 << 7 | 1 << 6)  // bit7=Read, bit6=Multi-byte


struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(adxl345), SPIOP, 0);

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

static int adxl345_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[] = { reg & 0x3F, val };  // Write command
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    return spi_write_dt(&spispec, &tx_set);
}

static int adxl345_read_reg(uint8_t reg, uint8_t *val)
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

#define ADXL345_DEVID_REG       0x00
#define ADXL345_POWER_CTL_REG   0x2D
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_BW_RATE_REG     0x2C

#define ADXL345_DEVID           0xCB
#define ADXL345_MEASURE         0x08
#define ADXL345_RANGE_2G        0x00
#define ADXL345_RATE_100HZ      0x0A  // Output Data Rate = 100 Hz
#define DELAY_VALUES 10  // Delay in milliseconds

int adxl345_init(void)
{
    if (!spi_is_ready_dt(&spispec)) {
        printk("SPI device not ready!\n");
        return -ENODEV;
    }

    uint8_t devid;
    int ret = adxl345_read_reg(ADXL345_DEVID_REG, &devid);
    if (ret != 0 || devid != ADXL345_DEVID) {
        printk("ADXL345 not detected! Read 0x%02X\n", devid);
        return -EIO;
    }

    printk("ADXL345 detected. DEVID: 0x%02X\n", devid);

    // Set BW_RATE to 100 Hz
    if (adxl345_write_reg(ADXL345_BW_RATE_REG, ADXL345_RATE_100HZ)) {
        printk("Failed to set BW_RATE\n");
        return -EIO;
    }

    // Set data format to ±2g, right-justified, full-res disabled
    if (adxl345_write_reg(ADXL345_DATA_FORMAT_REG, ADXL345_RANGE_2G)) {
        printk("Failed to set data format\n");
        return -EIO;
    }

    // Enable measurement mode
    if (adxl345_write_reg(ADXL345_POWER_CTL_REG, ADXL345_MEASURE)) {
        printk("Failed to enable measurement\n");
        return -EIO;
    }

    printk("ADXL345 initialization complete.\n");
    return 0;
}

int main(void)
{
	int err;
	
	err = spi_is_ready_dt(&spispec);
	if (!err) {
		LOG_ERR("Error: SPI device is not ready, err: %d", err);
		return 0;
	}

	LOG_INF("SPI device is ready ");

	if (adxl345_init() != 0) {
        printk("Sensor init failed\n");
        return -1;
    }

	int16_t x, y, z;

	while(1){
		/* STEP 10.4 - Continuously read sensor samples and toggle led */
		// bme_read_sample();

		if (adxl345_read_xyz(&x, &y, &z) == 0) {
            printk("X: %d  Y: %d  Z: %d\n", x, y, z);
        } else {
            printk("Failed to read XYZ\n");
        }

		// gpio_pin_toggle_dt(&ledspec);
		k_msleep(DELAY_VALUES);
	}

	return 0;
}
