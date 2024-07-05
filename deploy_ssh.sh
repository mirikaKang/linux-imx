#!/bin/bash
usage() {
    echo "사용법: $0 remote_address to deply"
    echo "예: $0 user hostname target_root"
    exit 1
}

# 인자 개수 확인
if [ "$#" -ne 3 ]; then
    usage
fi

user="$1"
host_name="$2"
target_root="$3"

output_folder="../linux-out"
kver=$(strings ${output_folder}/arch/arm64/boot/Image | grep -i "Linux version" | awk 'NR==1{print $3}')
echo "kernel version" $kver
deploy_target="${user}@${host_name}:/home/${user}/${target_root}"

#ssh dev01@127.0.0.1 mkdir -p /home/dev01/a/b/c
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}/boot
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}/sdk
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}/lib/modules

# deploy Image 
deploy_boot_target="${deploy_target}/boot/Image.gz-${kver}"
echo "copy boot image to" $deploy_boot_target
sudo scp -i /root/.ssh/id_rsa -P 22 -p ${output_folder}/arch/arm64/boot/Image.gz  ${deploy_boot_target}

# deploy DTB
deploy_dts="${deploy_target}/boot/."
echo "copy dts files to" ${deploy_dts}
sudo scp -i /root/.ssh/id_rsa -P 22 -p ${output_folder}/arch/arm64/boot/dts/freescale/*imx8mp*.dtb ${deploy_dts}

# deploy modules 
echo "module deploy"
modules_dir="${output_folder}/usr/modules/lib/modules/${kver}/"
echo "delete link files in mouduel folder"
find ${modules_dir} -type l -exec rm {} \;
deploy_modules_dir="${deploy_target}/lib/modules/"
echo "copy modules"${modules_dir} "to" ${deploy_modules_dir}
sudo scp -i /root/.ssh/id_rsa -P 22 -rp ${modules_dir} "${deploy_modules_dir}"
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}/lib/modules/${kver}/extra/video
sudo ssh ${user}@${host_name} mkdir -p /home/${user}/${target_root}/lib/modules/${kver}/extra/sensor