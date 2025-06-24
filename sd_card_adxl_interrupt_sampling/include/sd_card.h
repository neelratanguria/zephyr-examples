#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>

// Only declarations here!
extern FATFS fat_fs;
extern struct fs_file_t file;
extern struct fs_mount_t fatfs_mnt;
