diskutil list
diskutil unmountDisk /dev/disk5
sudo dd if=/dev/zero of=/dev/rdisk5 bs=512 count=2048
sudo newfs_msdos -F 32 -v SD /dev/rdisk5