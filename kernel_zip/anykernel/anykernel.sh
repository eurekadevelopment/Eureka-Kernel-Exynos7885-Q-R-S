# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() { '
kernel.string=Eureka Kernel by Eureka Team
do.devicecheck=0
do.modules=0
do.systemless=1
do.cleanup=1
do.cleanuponabort=0
device.name1=
device.name2=
device.name3=
device.name4=
device.name5=
supported.versions=
supported.patchlevels=
'; } # end properties

# shell variables
if [ -e /dev/block/platform/13500000.dwmmc0/by-name/BOOT ]; then
	block=/dev/block/platform/13500000.dwmmc0/by-name/BOOT;
elif [ -e /dev/block/platform/13500000.dwmmc0/by-name/boot ]; then
	block=/dev/block/platform/13500000.dwmmc0/by-name/boot;
fi

is_slot_device=0;
ramdisk_compression=auto;

## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;


## AnyKernel file attributes
# set permissions/ownership for included ramdisk files
set_perm_recursive 0 0 755 644 $ramdisk/*;
set_perm_recursive 0 0 750 750 $ramdisk/init* $ramdisk/sbin;


## AnyKernel install
#Method 1:
#dump_boot;
#write_boot;

#Method 2:
split_boot;
ui_print "- Installing Eureka kernel";
flash_boot;

ui_print " ";
ui_print "- Installing/updating Eureka dtb";
ui_print " ";
flash_dtb;

ui_print "- Installing/updating Eureka dtbo";
ui_print " ";
flash_dtbo;

## Copy additional files to internal storage
cp /tmp/anykernel/tools/espectrum.zip /data/media/0/enable_spectrum_support.zip;
chmod 755 /data/media/0/enable_spectrum_support.zip;

## Copy changelog to internal storage
cp /tmp/anykernel/tools/changelog.txt /data/media/0/changelog.txt;
chmod 755 /data/media/0/changelog.txt;

ui_print "- Installation finished successfully";
ui_print " ";

ui_print "- Thank you for using Eureka Kernel :)";
ui_print " ";

ui_print "- Flash zip found on your internal storage to";
ui_print "enable/update latest spectrum support";
ui_print " ";

## end install

