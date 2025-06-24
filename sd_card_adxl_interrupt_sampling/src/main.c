
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "sd_card.h"
#include "adxl345.h"

#include <zephyr/drivers/counter.h>

#define SAMPLE_INTERVAL_US 500  // 800 us = 1.745 kHz

K_SEM_DEFINE(sample_now_sem, 0, 1);

const struct device *counter_dev;


/*
 * SD Card SPI Pin Mapping
 * ------------------------
 * | SD Card Pin | nRF52 Pin |
 * |-------------|------------|
 * | CS          | P0.27      |
 * | SCK         | P0.23      |
 * | MOSI        | P0.20      |
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
 * | CS          | P0.04            | Controlled by software |
 * | SCL (SCLK)  | P0.31            | SPI clock              |
 * | SDA (MOSI)  | P0.30            | Data from nRF to ADXL  |
 * | SDO (MISO)  | P0.29            | Data from ADXL to nRF  |
 * | INT1        | P0.15 (optional) | If using interrupts    |
 * | INT2        | -                | Not used               |
 */

 LOG_MODULE_REGISTER(interrupt_sampling, LOG_LEVEL_INF);

 static void sample_timer_handler(const struct device *dev,
    uint8_t chan_id,
    uint32_t ticks,
    void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chan_id);
    ARG_UNUSED(ticks);
    ARG_UNUSED(user_data);

    k_sem_give(&sample_now_sem);
    // LOG_INF("Sample timer triggered, re-arming alarm");

    // Re-arm the alarm
    uint32_t now_ticks;
    counter_get_value(counter_dev, &now_ticks);

    struct counter_alarm_cfg alarm_cfg = {
        .ticks = counter_us_to_ticks(counter_dev, SAMPLE_INTERVAL_US) + now_ticks,
        .callback = sample_timer_handler,
        .user_data = NULL,
        .flags = COUNTER_ALARM_CFG_ABSOLUTE,
    };

    int err = counter_set_channel_alarm(counter_dev, chan_id, &alarm_cfg);
    if (err != 0) {
        LOG_ERR("Failed to re-set counter alarm: %d", err);
    }
}



// Buffer sizes and definitions

#define SAMPLE_SIZE 15  // 4 bytes k_cycle + 4 bytes uptime + 3x int16_t + 1 byte checksum = 15 bytes
#define BUFFER_SIZE (SAMPLE_SIZE * 100)  // 100 samples per buffer
#define DELAY_VALUES 1  // Delay in milliseconds

static uint8_t buffer_a[BUFFER_SIZE];
static uint8_t buffer_b[BUFFER_SIZE];

static uint8_t *active_buf = buffer_a;
static uint8_t *flush_buf = buffer_b;

static volatile size_t active_index = 0;
static struct k_sem flush_sem;  // Used to signal SD writer
static struct k_mutex buffer_swap_mutex;
static struct k_sem sd_ready_sem;


// Sampling Thread Configuration
#define STACK_SIZE 2048
#define SAMPLING_THREAD_PRIORITY 4

#define SD_WRITER_STACK_SIZE 2048
#define SD_WRITER_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(sampling_stack, STACK_SIZE);
static struct k_thread sampling_thread_data;

// Button interrupt configuration
#define BUTTON_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;
volatile bool button_pressed = false;

K_SEM_DEFINE(button_sem, 0, 1);

void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    button_pressed = true;
    LOG_INF("Button pressed! Pin: %d", button.pin);
    k_sem_give(&button_sem);
}

void sd_writer_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("SD writer thread started");
    k_sem_take(&sd_ready_sem, K_FOREVER);  // ✅ Wait until SD is ready
    LOG_INF("SD writer thread is ready to write");

    while (1) {
        k_sem_take(&flush_sem, K_FOREVER);

        int rc = fs_write(&file, flush_buf, BUFFER_SIZE);
        if (rc < 0) {
            LOG_ERR("Failed to write to SD: %d", rc);
        }

        // Sync the file to ensure data is written
        fs_sync(&file);  // Flush to disk after write
    }
}

// ✅ Start background SD writer thread
K_THREAD_DEFINE(sd_tid, SD_WRITER_STACK_SIZE, sd_writer_thread, NULL, NULL, NULL, SD_WRITER_THREAD_PRIORITY, 0, 0);


#define DISCARD_SAMPLE_COUNT 10

void sampling_thread(void *arg1, void *arg2, void *arg3)
{
    int16_t x, y, z;
    uint32_t sample_count = 0;
    int64_t start_time = k_uptime_get();

     // 🔁 Discard the first N samples (noise filter)
     for (int i = 0; i < DISCARD_SAMPLE_COUNT; ++i) {
        if (adxl345_read_xyz(&x, &y, &z) == 0) {
            // Optional: LOG_DBG("Discarded sample %d: X=%d Y=%d Z=%d", i, x, y, z);
        }
        k_msleep(DELAY_VALUES);  // or k_busy_wait(312) for 3.2 kHz
    }

    while (!button_pressed) {
        // LOG_INF("Waiting for sample timer...");
        k_sem_take(&sample_now_sem, K_FOREVER);  // Wait for timer
        if (adxl345_read_xyz(&x, &y, &z) == 0) {
            k_mutex_lock(&buffer_swap_mutex, K_FOREVER);
            

            // LOG_INF("Sample %d: X: %d, Y: %d, Z: %d", sample_count, x, y, z);
            sample_count++;

            uint32_t cycle_ts = k_cycle_get_32();      // High-resolution timestamp
            uint32_t uptime_ts = k_uptime_get_32();    // Millisecond timestamp

            // LOG_INF("Hit sample");
            if (active_index + SAMPLE_SIZE <= BUFFER_SIZE) {
                uint8_t temp_buf[14];  // 4 + 4 + 6 = 14 bytes for CRC calc
                int idx = 0;

                // Add timestamps
                temp_buf[idx++] = cycle_ts & 0xFF;
                temp_buf[idx++] = (cycle_ts >> 8) & 0xFF;
                temp_buf[idx++] = (cycle_ts >> 16) & 0xFF;
                temp_buf[idx++] = (cycle_ts >> 24) & 0xFF;

                temp_buf[idx++] = uptime_ts & 0xFF;
                temp_buf[idx++] = (uptime_ts >> 8) & 0xFF;
                temp_buf[idx++] = (uptime_ts >> 16) & 0xFF;
                temp_buf[idx++] = (uptime_ts >> 24) & 0xFF;

                // Add XYZ
                temp_buf[idx++] = x & 0xFF;
                temp_buf[idx++] = (x >> 8) & 0xFF;
                temp_buf[idx++] = y & 0xFF;
                temp_buf[idx++] = (y >> 8) & 0xFF;
                temp_buf[idx++] = z & 0xFF;
                temp_buf[idx++] = (z >> 8) & 0xFF;

                // Compute XOR checksum
                uint8_t checksum = 0;
                for (int i = 0; i < sizeof(temp_buf); i++) {
                    checksum ^= temp_buf[i];
                }

                // Copy to buffer
                for (int i = 0; i < sizeof(temp_buf); i++) {
                    active_buf[active_index++] = temp_buf[i];
                }

                active_buf[active_index++] = checksum;  // Append checksum
            } else {
                LOG_ERR("Buffer overflow! Active index: %d", active_index);
                // Optional: Trigger early flush
            }

            if (active_index >= BUFFER_SIZE) {
                uint8_t *temp = active_buf;
                active_buf = flush_buf;
                flush_buf = temp;
                active_index = 0;
                k_sem_give(&flush_sem);  // Signal writer thread
            }

            k_mutex_unlock(&buffer_swap_mutex);
        } else {
            LOG_ERR("Failed to read XYZ");
        }

        // For ~1 kHz sampling (adjust as needed)
        // k_msleep(DELAY_VALUES);  // or k_busy_wait(312) for 3.2 kHz
        // LOG_INF("Sample %d taken", sample_count);
    }

    int64_t elapsed = k_uptime_get() - start_time;
    LOG_INF("Samples per second: %d", (1000 * sample_count) / elapsed);
    LOG_INF("Total samples: %d", sample_count);
}

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
	
	err = check_spi_ready();
    if (err != 0) {
        LOG_ERR("SPI device not ready!");
        return;
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

    ret = fs_unlink("/SD:/test.bin");
    if (ret == 0) {
        LOG_INF("File deleted successfully.");
    } else {
        LOG_ERR("Failed to delete file: %d", ret);
    }

    fs_file_t_init(&file);
    rc = fs_open(&file, "/SD:/test.bin", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    

    k_sem_give(&sd_ready_sem);  // ✅ Signal SD writer thread to proceed
    LOG_INF("SD writer thread started and ready to write");

    // ✅ Init semaphores and mutexes
    k_sem_init(&flush_sem, 0, 1);
    k_mutex_init(&buffer_swap_mutex);

    counter_dev = DEVICE_DT_GET(DT_INST(0, nordic_nrf_rtc));  // rtc0

    if (!device_is_ready(counter_dev)) {
        LOG_ERR("Counter device not ready");
        return;
    }

    struct counter_alarm_cfg alarm_cfg = {
        .ticks = counter_us_to_ticks(counter_dev, SAMPLE_INTERVAL_US),
        .callback = sample_timer_handler,
        .user_data = NULL,
        .flags = COUNTER_ALARM_CFG_ABSOLUTE,
    };

    err = counter_start(counter_dev);
    if (err != 0) {
        LOG_ERR("Failed to start counter: %d", err);
        return;
    }

    err = counter_set_channel_alarm(counter_dev, 0, &alarm_cfg);
    if (err != 0) {
        LOG_ERR("Failed to set counter alarm: %d", err);
        return;
    }

    LOG_INF("Sampling timer started at %d us interval", SAMPLE_INTERVAL_US);


    // Create the sampling thread
    k_thread_create(&sampling_thread_data, sampling_stack,
        K_THREAD_STACK_SIZEOF(sampling_stack),
        sampling_thread,
        NULL, NULL, NULL,
        SAMPLING_THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("Sampling thread started");

    // Wait forever until button is pressed
    k_sem_take(&button_sem, K_FOREVER);

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
    // adxl345_write_reg(ADXL345_POWER_CTL_REG, 0x00); // Power down
    // LOG_INF("ADXL345 sensor powered down.");
    // Optionally, you can also reset the SPI device
    release_spi();
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