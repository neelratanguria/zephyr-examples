
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

static FATFS fat_fs;

static struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data = &fat_fs,
    .storage_dev = (void *)DEVICE_DT_GET(DT_NODELABEL(sdcard)),
};


void main(void)
{
    int rc = disk_access_init("SD");
    if (rc != 0) {
        printk("Disk init error %d\n", rc);
        return;
    }

    memset(&fat_fs, 0, sizeof(fat_fs));

    // delay to allow the SD card to stabilize
    k_msleep(1000);

    const struct device *sd_dev = DEVICE_DT_GET(DT_NODELABEL(sdcard));
    if (!device_is_ready(sd_dev)) {
        printk("SD card device not ready!\n");
        return;
    }

    
    rc = fs_mount(&fatfs_mnt);
    if (rc == 0) {
        printk("SD card mounted at %s\n", fatfs_mnt.mnt_point);
    } else {
        printk("Failed to mount SD card: %d\n", rc);
        return;
    }

    struct fs_file_t file;
    fs_file_t_init(&file);
    rc = fs_open(&file, "/SD:/test.txt", FS_O_CREATE | FS_O_WRITE);
    fs_write(&file, "Hello SD!", 9);
    fs_close(&file);
}