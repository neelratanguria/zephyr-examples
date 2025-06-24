#include "sd_card.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sd_card, LOG_LEVEL_DBG);

// ✅ Now this is the only place where they're defined
FATFS fat_fs;
struct fs_file_t file;

struct fs_mount_t fatfs_mnt = {
    .type = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data = &fat_fs,
    .storage_dev = (void *)DEVICE_DT_GET(DT_NODELABEL(sdcard)),
};
