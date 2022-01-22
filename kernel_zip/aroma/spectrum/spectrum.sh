VAR=$1

## AnyKernel install
mount_androidroot() {
	export TMPSYSROOT=$ANDROID_ROOT
	mount $ANDROID_ROOT
}

mount_system() {
	export TMPSYSROOT=/system
	mount /system
}

mount_systemroot() {
	export TMPSYSROOT=/system_root
	mount /system_root
}

mount_systemroot || mount_system || mount_androidroot
mount /vendor

# Remount system as read-write
mount -o rw,remount -t auto $TMPSYSROOT
mount -o rw,remount -t auto /vendor

# Locate init path
INIT_PATH=$TMPSYSROOT/system/etc/init
INIT_PATH2=/vendor/etc/init

if [ -d "$INIT_PATH" ]; then
	INIT=$INIT_PATH
elif [ -d "$INIT_PATH2" ]; then
	INIT=$INIT_PATH2
fi

# Remove old spectrum
if [ -e "$INIT/init.spectrum.rc" ]; then
	rm -f $INIT/init.spectrum.rc
	rm -f $INIT/init.spectrum.sh
fi

if [ -e "$TMPSYSROOT/init.spectrum.rc" ]; then
	rm -f $TMPSYSROOT/init.spectrum.rc
	rm -f $TMPSYSROOT/init.spectrum.sh
fi

cp -f /tmp/spectrum/$VAR/init.spectrum.rc $INIT/
chmod 644 $INIT/init.spectrum.rc
cp -f /tmp/spectrum/init.spectrum.sh $INIT/
chmod 755 $INIT/init.spectrum.sh

## end install
