
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>

LOG_MODULE_REGISTER(Lesson5_Exercise1, LOG_LEVEL_INF);

// Buffer sizes and definitions

#define SAMPLE_SIZE 6    // 3x int16_t = 6 bytes
#define BUFFER_SIZE 600

static uint8_t buffer_a[BUFFER_SIZE];
static uint8_t buffer_b[BUFFER_SIZE];

static uint8_t *active_buf = buffer_a;
static uint8_t *flush_buf = buffer_b;

static volatile size_t active_index = 0;
static struct k_sem flush_sem;  // Used to signal SD writer
static struct k_mutex buffer_swap_mutex;
static struct k_sem sd_ready_sem;

/*
 * SD Card SPI Pin Mapping
 * ------------------------
 * | SD Card Pin | nRF52 Pin |
 * |-------------|------------|
 * | CS          | P0.27      |
 * | SCK         | P0.23      |
 * | MOSI        | P0.02      |
 * | MISO        | P0.26      |
 * | VCC         | 3.3V       |
 * | GND         | GND        |
 *
 * Note:
 * - Ensure VCC is a stable 3.3V (not 3.0V or lower).
 * - Common ground between SD card and nRF52833 is essential.
 * - Pull-up resistors on SD lines are usually not needed for SPI mode.
 */

 /*
 * ADXL345 to nRF52833 SPI Pin Mapping
 *
 * | ADXL345 Pin | nRF52833 Pin     | Notes                  |
 * |-------------|------------------|------------------------|
 * | VCC         | 3.3V             | Use VDD from nRF52833  |
 * | GND         | GND              | Common ground          |
 * | CS          | P0.28            | Controlled by software |
 * | SCL (SCLK)  | P0.31            | SPI clock              |
 * | SDA (MOSI)  | P0.30            | Data from nRF to ADXL  |
 * | SDO (MISO)  | P0.29            | Data from ADXL to nRF  |
 * | INT1        | P0.15 (optional) | If using interrupts    |
 * | INT2        | -                | Not used               |
 */


// Button interrupt configuration
#define BUTTON_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;
volatile bool button_pressed = false;

void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    button_pressed = true;
    LOG_INF("Button pressed! Pin: %d", button.pin);
}



static FATFS fat_fs;
static struct fs_file_t file;


static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data = &fat_fs,
    .storage_dev = (void *)DEVICE_DT_GET(DT_NODELABEL(sdcard)),
};

// ADXL345 SPI configuration

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

#define ADXL345_DEVID           0xE5
// #define ADXL345_DEVID           0xCB
#define ADXL345_MEASURE         0x08
#define ADXL345_RANGE_2G        0x00
#define ADXL345_RATE_100HZ      0x0A  // Output Data Rate = 100 Hz
#define DELAY_VALUES 100  // Delay in milliseconds

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

void sd_writer_thread(void *p1, void *p2, void *p3)
{
    k_sem_take(&sd_ready_sem, K_FOREVER);  // ✅ Wait until SD is ready

    while (1) {
        k_sem_take(&flush_sem, K_FOREVER);

        

        int rc = fs_write(&file, flush_buf, BUFFER_SIZE);
        if (rc < 0) {
            LOG_ERR("Failed to write to SD: %d", rc);
        }
        LOG_INF("Wrote %d bytes to SD", rc);
        // Sync the file to ensure data is written
        fs_sync(&file);  // Flush to disk after write
        LOG_INF("Flushed file to SD");
    }
}

// ✅ Start background SD writer thread
K_THREAD_DEFINE(sd_tid, 1024, sd_writer_thread, NULL, NULL, NULL, 5, 0, 0);

void main(void)
{
    k_sem_init(&sd_ready_sem, 0, 1);

    if (!device_is_ready(button.port)) {
        LOG_ERR("Button device %s is not ready", button.port->name);
        return;
    }
    
    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure button pin: %d", ret);
        return;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure interrupt on button pin: %d", ret);
        return;
    }
    
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    
    LOG_INF("Button interrupt configured");

    int err;
	
	err = spi_is_ready_dt(&spispec);
	if (!err) {
		LOG_ERR("Error: SPI device is not ready, err: %d", err);
		return 0;
	}

	LOG_INF("SPI device is ready ");

    if (adxl345_init() != 0) {
        LOG_ERR("Sensor init failed\n");
        // return -1;
    }

	


    int rc = disk_access_init("SD");
    if (rc != 0) {
        LOG_ERR("Disk init error %d\n", rc);
        return;
    }

    memset(&fat_fs, 0, sizeof(fat_fs));

    // delay to allow the SD card to stabilize
    k_msleep(1000);

    const struct device *sd_dev = DEVICE_DT_GET(DT_NODELABEL(sdcard));
    if (!device_is_ready(sd_dev)) {
        LOG_ERR("SD card device not ready!\n");
        return;
    }

    
    rc = fs_mount(&fatfs_mnt);
    if (rc == 0) {
        LOG_INF("SD card mounted at %s\n", fatfs_mnt.mnt_point);
    } else {
        LOG_ERR("Failed to mount SD card: %d\n", rc);
        return;
    }

    
    fs_file_t_init(&file);
    rc = fs_open(&file, "/SD:/test.bin", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    // fs_write(&file, "Hello SD!", 9);
    // fs_close(&file);

    k_sem_give(&sd_ready_sem);  // ✅ Signal SD writer thread to proceed

    // ✅ Init semaphores and mutexes
    k_sem_init(&flush_sem, 0, 1);
    k_mutex_init(&buffer_swap_mutex);


    int16_t x, y, z;

    while (!button_pressed) {
        if (adxl345_read_xyz(&x, &y, &z) == 0) {
            LOG_INF("X: %d  Y: %d  Z: %d", x, y, z);
    
            k_mutex_lock(&buffer_swap_mutex, K_FOREVER);
    
            // Check if enough space
            if (active_index + SAMPLE_SIZE <= BUFFER_SIZE) {
                active_buf[active_index++] = x & 0xFF;
                active_buf[active_index++] = (x >> 8) & 0xFF;
                active_buf[active_index++] = y & 0xFF;
                active_buf[active_index++] = (y >> 8) & 0xFF;
                active_buf[active_index++] = z & 0xFF;
                active_buf[active_index++] = (z >> 8) & 0xFF;
                LOG_INF("Active index: %d", active_index);
            } else {
                LOG_ERR("Buffer overflow! Active index: %d", active_index);
                // Trigger SD write immediately
            }

            
    
            // If full, trigger SD write
            if (active_index >= BUFFER_SIZE) {
                // for (int i = 0; i < BUFFER_SIZE; i += 6) {
                //     int16_t x = (int16_t)(flush_buf[i] | (flush_buf[i+1] << 8));
                //     int16_t y = (int16_t)(flush_buf[i+2] | (flush_buf[i+3] << 8));
                //     int16_t z = (int16_t)(flush_buf[i+4] | (flush_buf[i+5] << 8));
                //     LOG_INF("Sample %03d: X=%d Y=%d Z=%d", i / 6, x, y, z);
                // }

                uint8_t *temp = active_buf;
                active_buf = flush_buf;
                flush_buf = temp;
                active_index = 0;
    
                k_sem_give(&flush_sem);  // notify SD write thread
            }
    
            k_mutex_unlock(&buffer_swap_mutex);
    
        } else {
            LOG_ERR("Failed to read XYZ");
        }
    
        k_msleep(DELAY_VALUES);
    }


    LOG_INF("Button pressed, exiting main loop.");
    fs_unmount(&fatfs_mnt);
    LOG_INF("Unmounted SD card.");
    gpio_remove_callback(button.port, &button_cb_data);
    LOG_INF("Button callback removed.");
    // Optionally, you can also close the file if it was opened
    fs_file_t_init(&file);
    fs_close(&file);
    LOG_INF("File closed.");
    // Optionally, you can also unmount the filesystem
    // fs_unmount(&fatfs_mnt);
    LOG_INF("Filesystem unmounted.");
    // Optionally, you can also reset the ADXL345 sensor
    adxl345_write_reg(ADXL345_POWER_CTL_REG, 0x00); // Power down
    LOG_INF("ADXL345 sensor powered down.");
    // Optionally, you can also reset the SPI device
    spi_release_dt(&spispec);
    LOG_INF("SPI device released.");
    // Optionally, you can also reset the SD card device
    const struct device *sdcard_dev = DEVICE_DT_GET(DT_NODELABEL(sdcard));
    if (device_is_ready(sdcard_dev)) {
        // Reset or reinitialize the SD card device if needed
        LOG_INF("SD card device is ready for reset.");
    } else {
        LOG_ERR("SD card device is not ready for reset.");
    }
    LOG_INF("Main function completed.");
    // Optionally, you can also reset the button device
    const struct device *button_dev = button.port;
    if (device_is_ready(button_dev)) {
        // Reset or reinitialize the button device if needed
        LOG_INF("Button device is ready for reset.");
    } else {
        LOG_ERR("Button device is not ready for reset.");
    }
    LOG_INF("Button device reset completed.");
    // Optionally, you can also reset the GPIO subsystem
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (device_is_ready(gpio_dev)) {
        // Reset or reinitialize the GPIO subsystem if needed
        LOG_INF("GPIO subsystem is ready for reset.");
    } else {
        LOG_ERR("GPIO subsystem is not ready for reset.");
    }
    LOG_INF("GPIO subsystem reset completed.");
    
}