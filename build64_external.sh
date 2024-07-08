#!/bin/bash
mkdir -p ../linux-out/
mkdir -p ../linux-out/usr/modules
make O=../linux-out imx8mp_var_dart_nodev_defconfig 
# make O=../linux-out imx8mp-var-dart-nodev-rev0-defconfig 
# make O=../linux-out imx8_var_defconfig 
make O=../linux-out LOCALVERSION= Image -j4
make O=../linux-out LOCALVERSION= dtbs
make O=../linux-out LOCALVERSION= modules -j4
make O=../linux-out LOCALVERSION= modules_install INSTALL_MOD_PATH=../linux-out/usr/modules INSTALL_MOD_STRIP=1
gzip -c ../linux-out/arch/arm64/boot/Image  > ../linux-out/arch/arm64/boot/Image.gz
