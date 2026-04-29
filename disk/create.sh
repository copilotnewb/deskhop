
#!/bin/bash
set -e

dd if=/dev/zero of=disk.img bs=512 count=128

mkdosfs  -F12 -n DESKHOP -i 0 disk.img

sudo mount -o loop,x-mount.mkdir -t vfat disk.img /mnt/disk/
sudo cp ../webconfig/config.htm /mnt/disk/config.htm
sudo umount /mnt/disk
