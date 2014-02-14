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
mkdir out
cp arch/arm/boot/zImage out/zImage
cp drivers/scsi/scsi_wait_scan.ko out/scsi_wait_scan.ko
cp drivers/usb/serial/baseband_usb_chr.ko out/baseband_usb_chr.ko
cp crypto/tcrypt.ko out/tcrypt.ko
cp drivers/net/usb/raw_ip_net.ko out/raw_ip_net.ko
cp out/* ~/smb/kernel/out/
echo 'done'
echo ''
echo '#############'
echo 'build finished successfully'
echo '#############'
