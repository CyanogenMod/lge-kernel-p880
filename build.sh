#!/bin/sh

###### defines ######

local_dir=$PWD

###### defines ######
echo '#############'
echo 'making clean'
echo '#############'
make clean
rm -rf out
echo '#############'
echo 'making defconfig'
echo '#############'
make cyanogenmod_x3_defconfig
echo '#############'
echo 'making zImage'
echo '#############'
time make -j32
echo '#############'
echo 'copying files to ./out'
echo '#############'
echo ''
mkdir out
mkdir out/modules
cp arch/arm/boot/zImage out/zImage
cp drivers/scsi/scsi_wait_scan.ko out/modules/scsi_wait_scan.ko
cp drivers/usb/serial/baseband_usb_chr.ko out/modules/baseband_usb_chr.ko
cp crypto/tcrypt.ko out/modules/tcrypt.ko
cp drivers/net/usb/raw_ip_net.ko out/modules/raw_ip_net.ko
cp -r out/* ~/smb/kernel/out/
echo 'done'
echo ''
echo '#############'
echo 'Making Anykernel zip'
echo '#############'
echo ''
cd ~/smb/kernel/out/
. pack_cwm.sh
cd $local_dir
echo ''
echo '#############'
echo 'build finished successfully'
echo '#############'
