# Samsung arm Chromebook kernel (XE303C12 Snow)
This is fork of [Google Chromeos kernel](https://chromium.googlesource.com/chromiumos/third_party/kernel/) with some small patches like as enabled swap partition. In chromeos's kernel this function was disabled.

###Branches:
 * [chromeos-3.8](https://github.com/singulared/chromebook-kernel/tree/chromeos-3.8) - Google chromeos main branch
 * [chromeos-3.8-mali](https://github.com/singulared/chromebook-kernel/tree/chromeos-3.8-mali) - Google chromeos-3.8 branch with latest mali patches for [mali-t6xx-gpu userspace blobs](http://malideveloper.arm.com/develop-for-mali/features/mali-t6xx-gpu-user-space-drivers/). 

###Building signed chromeos image:
    make -j2 zImage dtbs modules
    make modules_install INSTALL_MOD_PATH=../modules```
    mkimage -f kernel.its kernel.itb
Pack image and sign:

    vbutil_kernel --pack kernel.img --keyblock /usr/share/vboot/devkeys/kernel.keyblock --version 1 --signprivate /usr/share/vboot/devkeys/kernel_data_key.vbprivk --config=cmdline --vmlinuz kernel/kernel.itb --arch arm
For packing you should install some package:

    apt-get install vboot-kernel-utils u-boot-tools

and cmdline file should contain something like:

    console=tty1 verbose root=/dev/mmcblk0p3 rootwait rw rootfstype=ext4

###Building uboot image:
    make -j2 uImage dtbs modules

And you may use `arch/arm/boot/uImage` with [nv-uboot](http://www.chromium.org/chromium-os/u-boot-porting-guide/using-nv-u-boot-on-the-samsung-arm-chromebook).
