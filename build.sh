#!/bin/bash
#
# Custom build script for Eureka kernels by Chatur27 and Gabriel260 @Github - 2020
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

# Set default directories
ROOT_DIR=$(pwd)
# OUT_DIR=$ROOT_DIR/out
KERNEL_DIR=$ROOT_DIR
DTBO_DIR=./arch/arm64/boot/dts/exynos/dtbo

# Set default kernel variables
PROJECT_NAME="Eureka Kernel"
CORES=$(nproc --all)
SELINUX_STATUS=""
REV=7.5
RR=0
PCUSER=chatur
USER=Chatur
ZIPNAME=Eureka_Rx.x_Axxx_xxxx_x_x.zip
DEFAULT_NAME=Eureka_Rx.x_Axxx_P/Q/R
GCC_ARM64_FILE=aarch64-linux-gnu-
GCC_ARM32_FILE=arm-linux-gnueabi-
CLANGC="nope"

# Export commands
export KBUILD_BUILD_USER=$USER
export KBUILD_BUILD_HOST=Eureka.org
export VERSION=$DEFAULT_NAME
export ARCH=arm64
export CROSS_COMPILE=$(pwd)/toolchain/bin/$GCC_ARM64_FILE
export CROSS_COMPILE_ARM32=$(pwd)/toolchain/bin/$GCC_ARM32_FILE

# Get date and time
DATE=$(date +"%m-%d-%y")
BUILD_START=$(date +"%s")

####################### Devices List #########################

SM_A105X() {

	DEVICE_Axxx=A105
	DEFCONFIG=exynos7885-a10_"$SELINUX_STATUS"defconfig
}

SM_A205X() {
	DEVICE_Axxx=A205
	DEFCONFIG=exynos7885-a20_"$SELINUX_STATUS"defconfig
}

SM_A202X() {
	DEVICE_Axxx=A202
	DEFCONFIG=exynos7885-a20e_"$SELINUX_STATUS"defconfig
}

SM_A305X() {
	DEVICE_Axxx=A305
	DEFCONFIG=exynos7885-a30_"$SELINUX_STATUS"defconfig
}

SM_A307X() {
	DEVICE_Axxx=A307
	DEFCONFIG=exynos7885-a30s_"$SELINUX_STATUS"defconfig
}

SM_A405X() {
	DEVICE_Axxx=A405
	DEFCONFIG=exynos7885-a40_"$SELINUX_STATUS"defconfig
}

SM_M205X() {
	DEVICE_Axxx=M205
	DEFCONFIG=exynos7885-m20_"$SELINUX_STATUS"defconfig
}

################################################################

######################## Android OS list #######################

androidp="Android 9 (Pie)"
androidqr="Android 10 (Q) 11 (R)"
androidr="Android 11 (oneui 3)"

################################################################

################### Executable functions #######################
CLEAN_SOURCE() {
	echo "*****************************************************"
	echo " "
	echo "              Cleaning kernel source"
	echo " "
	echo "*****************************************************"

	if [ -e "arch/arm64/boot/dtbo.img" ]; then
		{
			rm $DTBO_DIR/*.dtbo
		}
	fi

	make clean
	CLEAN_SUCCESS=$?
	if [ $CLEAN_SUCCESS != 0 ]; then
		echo " Error: make clean failed"
		exit
	fi

	make mrproper
	MRPROPER_SUCCESS=$?
	if [ $MRPROPER_SUCCESS != 0 ]; then
		echo " Error: make mrproper failed"
		exit
	fi

	if [ -e "kernel_zip/anykernel/Image" ]; then
		{
			rm $DTBO_DIR/*.dtbo
			rm -rf kernel_zip/anykernel/Image
			rm -rf kernel_zip/anykernel/dtbo.img
		}
	fi

	ADDITIONAL_CLEANUP
	sleep 1
}

CLANG_CLEAN() {
	echo "*****************************************************"
	echo " "
	echo "              Cleaning kernel source"
	echo " "
	echo "*****************************************************"

	rm -rf out
	if [ -e "kernel_zip/anykernel/Image" ]; then
		{
			rm -rf arch/$ARCH/boot/Image
			rm -rf arch/$ARCH/boot/dtbo.img
			rm -rf arch/$ARCH/boot/dtb.img
			rm -rf kernel_zip/anykernel/Image
			rm -rf kernel_zip/anykernel/dtbo.img
			rm -rf kernel_zip/anykernel/dtb.img
		}
	fi

	ADDITIONAL_CLEANUP
}

ADDITIONAL_CLEANUP() {
	if [ -e "drivers/usb/gadget/.oneui_mtp" ]; then
		rm -rf drivers/usb/gadget/.oneui_mtp
	fi

	if [ -e "drivers/usb/gadget/.gsi_mtp" ]; then
		rm -rf drivers/usb/gadget/.gsi_mtp
	fi

	if [ -e "security/selinux/.permissive" ]; then
		rm -rf security/selinux/.permissive
	fi

	if [ -e "security/selinux/.enforcing" ]; then
		rm -rf security/selinux/.enforcing
	fi
}

BUILD_KERNEL() {
	echo "*****************************************************"
	echo "      Building kernel for $DEVICE_Axxx android $ANDROID"
	export ANDROID_MAJOR_VERSION=$ANDROID
	export PLATFORM_VERSION=$AND_VER
	export LOCALVERSION=-$VERSION
	make $DEFCONFIG
	make -j$CORES
	sleep 1
}

AUTO_TOOLCHAIN() {
	if [ -e "toolchain/linaro6.5" ]; then
		{
			echo " "
			echo "Using Linaro v6.5.0 toolchain"
			echo " "
			GCC_ARM64_FILE=aarch64-linux-gnu-
			GCC_ARM32_FILE=arm-linux-gnueabi-
			export CROSS_COMPILE=$(pwd)/toolchain/bin/$GCC_ARM64_FILE
			export CROSS_COMPILE_ARM32=$(pwd)/toolchain/bin/$GCC_ARM32_FILE
		}
	elif [ -e "toolchain/gcc4.9" ]; then
		{
			echo " "
			echo "Using Gcc v4.9 toolchain"
			echo " "
			GCC_ARM64_FILE=aarch64-linux-android-
			GCC_ARM32_FILE=arm-linux-gnueabi-
			export CROSS_COMPILE=$(pwd)/toolchain/bin/$GCC_ARM64_FILE
			export CROSS_COMPILE_ARM32=$(pwd)/toolchain/bin/$GCC_ARM32_FILE
		}
	elif [ -e "toolchain/pro_clang13" ]; then
		{
			echo " "
			echo "Using Proton Clang 13 as main compiler"
			echo " "
			GCC_ARM64_FILE=aarch64-linux-gnu-
			GCC_ARM32_FILE=arm-linux-gnueabi-
			export CLANGC="OK"
			echo " "
		}
	else
		echo " "
		echo "WARNING: Toolchain directory couldn't be found"
		echo " "
		sleep 2
		exit
	fi
}

CLANG() {
	export LOCALVERSION=-$VERSION
	export PLATFORM_VERSION=$AND_VER
	make O=out ARCH=arm64 ANDROID_MAJOR_VERSION=$ANDROID $DEFCONFIG
	PATH="$KERNEL_DIR/toolchain/bin:$KERNEL_DIR/toolchain/bin:${PATH}" \
		make -j$CORES O=out \
		ARCH=arm64 \
		ANDROID_MAJOR_VERSION=$ANDROID \
		CC=clang \
		LD_LIBRARY_PATH="$KERNEL_DIR/toolchain/lib:$LD_LIBRARY_PATH" \
		CLANG_TRIPLE=aarch64-linux-gnu- \
		CROSS_COMPILE=$GCC_ARM64_FILE \
		CROSS_COMPILE_ARM32=$GCC_ARM32_FILE
}

PREBUILT_DTBO() {
	# Copy prebuilt dtbo.img to anykernel directory
	if [ "${DEVICE_Axxx}" == "A105" ]; then
		cp -f kernel_zip/dtbo/a10/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "A205" ]; then
		cp -f kernel_zip/dtbo/a20/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "A202" ]; then
		cp -f kernel_zip/dtbo/a20e/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "A305" ]; then
		cp -f kernel_zip/dtbo/a30/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "A307" ]; then
		cp -f kernel_zip/dtbo/a30s/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "A405" ]; then
		cp -f kernel_zip/dtbo/a40/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${DEVICE_Axxx}" == "M205" ]; then
		cp -f kernel_zip/dtbo/m20/dtbo.img kernel_zip/anykernel/dtbo.img
	fi
}

ZIPPIFY() {
	# Make Eureka flashable zip

	if [ -e "arch/$ARCH/boot/Image" ]; then
		{
			echo -e "*****************************************************"
			echo -e "                                                     "
			echo -e "       Building Eureka anykernel flashable zip       "
			echo -e "                                                     "
			echo -e "*****************************************************"

			# Copy Image and dtbo.img to anykernel directory
			cp -f arch/$ARCH/boot/Image kernel_zip/anykernel/Image
			cp -f arch/$ARCH/boot/dtb.img kernel_zip/anykernel/dtb.img
			PREBUILT_DTBO
			#cp -f arch/$ARCH/boot/dtbo.img kernel_zip/anykernel/dtbo.img

			# Go to anykernel directory
			cd kernel_zip/anykernel
			zip -r9 $ZIPNAME META-INF tools anykernel.sh Image dtb.img dtbo.img version
			chmod 0777 $ZIPNAME
			# Change back into kernel source directory
			cd ..
			sleep 1
			cd ..
			sleep 1
		}
	fi
}

PROCESSES() {
	# Allow user to choose how many cores to be taken by compiler
	echo "Your system has $CORES cores."
	read -p "Please enter how many cores to be used by compiler (Leave blank to use all cores) : " cores
	if [ "${cores}" == "" ]; then
		echo " "
		echo "Using all $CORES cores for compilation"
		sleep 1
	else
		echo " "
		echo "Using $cores cores for compilation "
		CORES=$cores
		sleep 1
	fi
}

ENTER_VERSION() {
	# Enter kernel revision for this build.
	read -p "Please type kernel version without R (E.g: 6.5) : " rev
	if [ "${rev}" == "" ]; then
		echo " "
		echo "     Using '$REV' as version"
	else
		REV=$rev
		echo " "
		echo "     Version = $REV"
	fi
	sleep 1
}

USER() {
	# Setup KBUILD_BUILD_USER
	echo " Current build username is $USER"
	echo " "
	read -p " Please type build_user (E.g: Chatur) : " user
	if [ "${user}" == "" ]; then
		echo " "
		echo "     Using '$USER' as Username"
	else
		export KBUILD_BUILD_USER=$user
		USER=$user
		echo " "
		echo "     build_user = $user"
	fi
	sleep 1
}

RENAME() {
	# Give proper name to kernel and zip name
	if [ "$RR" == "1" ]; then
		VERSION="Eureka_R"$REV"_"$DEVICE_Axxx"_"$SELINUX_STATUS"OneUI3/4"
		ZIPNAME="Eureka_R"$REV"_"$DEVICE_Axxx"_"$SELINUX_STATUS"OneUI3.zip"
	else
		VERSION="Eureka_R"$REV"_"$DEVICE_Axxx"_"$SELINUX_STATUS"Q/R/S"
		ZIPNAME="Eureka_R"$REV"_"$DEVICE_Axxx"_"$SELINUX_STATUS"Q_R_S.zip"
	fi
}

SELINUX() {
	# setup selinux for differents firmwares
	echo -e "***************************************************************"
	echo "             Select which version you wish to build               "
	echo -e "***************************************************************"
	echo " Available versions:"
	echo " "
	echo "  1. Build Eureka with ENFORCING SElinux"
	echo " "
	echo "  2. Build Eureka with PERMISSIVE SElinux"
	echo " "
	echo "  3. Leave empty to exit this script"
	echo " "
	echo " "
	read -n 1 -p "Select your choice: " -s choice
	case ${choice} in
	1)
		{
			export SELINUX_B=enforcing
			export SELINUX_STATUS="$SELINUX_B"_
		}
		;;
	2)
		{
			export SELINUX_B=permissive
			export SELINUX_STATUS="$SELINUX_B"_
		}
		;;
	*)
		{
			echo
			echo "Invalid choice entered. Exiting..."
			sleep 2
			exit 1
		}
		;;
	esac
	sleep 1
}

ONEUI_STATE() {
	# Since wireguard checks for update during compilation, its group will change from $user to root.
	# So change it back to default user group. Do this only for me. Gabriel does not need that i guess.
	if [ ${USER} == "Chatur" ]; then
		{
			chown -R $PCUSER $(pwd)/net/wireguard
		}
	fi
}

DISPLAY_ELAPSED_TIME() {
	# Find out how much time build has taken
	BUILD_END=$(date +"%s")
	DIFF=$(($BUILD_END - $BUILD_START))

	BUILD_SUCCESS=$?
	if [ $BUILD_SUCCESS != 0 ]; then
		echo " Error: Build failed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds $reset"
		exit
	fi

	echo -e "                     Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds $reset"
	sleep 1
}

COMMON_STEPS() {
	echo "*****************************************************"
	echo "                                                     "
	echo "        Starting compilation of $DEVICE_Axxx kernel  "
	echo "                                                     "
	echo " Defconfig = $DEFCONFIG                              "
	echo "                                                     "
	echo "*****************************************************"
	RENAME
	sleep 1
	echo " "
	if [ ${CLANGC} == "OK" ]; then
		{
			CLANG
		}
	else
		{
			BUILD_KERNEL
		}
	fi
	echo " "
	sleep 1
	if [ ${CLANGC} == "OK" ]; then
		{
			cp -f out/arch/$ARCH/boot/Image arch/$ARCH/boot/Image
			cp -f out/arch/$ARCH/boot/dtb.img arch/$ARCH/boot/dtb.img
			cp -f out/arch/$ARCH/boot/dtbo.img arch/$ARCH/boot/dtbo.img
		}
	fi
	ZIPPIFY
	sleep 1
	if [ ${CLANGC} == "OK" ]; then
		{
			CLANG_CLEAN
		}
	else
		{
			CLEAN_SOURCE
		}
	fi
	sleep 1
	echo " "
	DISPLAY_ELAPSED_TIME
	echo " "
	echo "                 *****************************************************"
	echo "*****************                                                     *****************"
	echo "                      $DEVICE_Axxx kernel for Android $AND_VER build finished          "
	echo "*****************                                                     *****************"
	echo "                 *****************************************************"
}

OS_MENU() {
	# Give the choice to choose Android Version
	PS3='
Please select your Android Version: '
	menuos=("$androidp" "$androidqr" "$androidr" "Exit")
	select menuos in "${menuos[@]}"; do
		case $menuos in
		"$androidp")
			echo " "
			echo "Android 9 (Pie) chosen as Android Major Version"
			ANDROID=p
			AND_VER=9
			sleep 2
			echo " "
			break
			;;
		"$androidqr")
			echo " "
			echo "Android 10 (Q) / 11 (R) chosen as Android Major Version"
			ANDROID=r
			AND_VER=11
			sleep 2
			echo " "
			break
			;;
		"$androidr")
			echo " "
			echo "Android 11 (OneUI 3) chosen as Android Major Version"
			ANDROID=r
			AND_VER=11
			RR=1
			sleep 2
			echo " "
			break
			;;
		"Exit")
			echo " "
			echo "Exiting build script.."
			sleep 2
			echo " "
			exit
			;;
		*)
			echo Invalid option.
			;;
		esac
	done
}

#################################################################

###################### Script starts here #######################

AUTO_TOOLCHAIN
if [ ${CLANGC} == "OK" ]; then
	{
		CLANG_CLEAN
		sleep 2
	}
else
	{
		CLEAN_SOURCE
	}
fi
clear
PROCESSES
clear
ENTER_VERSION
clear
USER
# Disable updating oneui build_files for the time being..
#clear
#UPDATE_BUILD_FILES
clear
SELINUX
clear
echo "******************************************************"
echo "*             $PROJECT_NAME Build Script             *"
echo "*                  Developer: Chatur                 *"
echo "*                Co-Developer: Gabriel               *"
echo "*                                                    *"
if [ ${CLANGC} == "OK" ]; then
	echo "*      Compiling kernel using Proton Clang 13        *"
else
	echo "*        Compiling kernel using gay toochain         *"
fi
echo "*                                                    *"
echo "******************************************************"
echo " Some informations about parameters set:		"
echo -e "    > Architecture: $ARCH				"
echo "    > Jobs: $CORES					"
echo "    > Revision for this build: R$REV			"
echo "    > SElinux Status: $SELINUX_B			"
echo "    > Kernel Name Template: $VERSION			"
echo "    > Build user: $KBUILD_BUILD_USER			"
echo "    > Build machine: $KBUILD_BUILD_HOST		"
echo "    > Build started on: $BUILD_START			"
echo "    > ARM64 Toolchain exported				"
echo "    > ARM32 Toolchain exported				"
echo -e "*****************************************************"
echo " "

echo " Devices avalaible for compilation: "
echo " "
PS3='
 Please select your device: '
menuoptions=("SM_A105X" "SM_A205X" "SM_A202X" "SM_A305X" "SM_A307X" "SM_A405X" "SM_M205X" "Exit")
select menuoptions in "${menuoptions[@]}"; do
	case $menuoptions in
	"SM_A105X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A105X
		COMMON_STEPS
		break
		;;
	"SM_A205X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A205X
		COMMON_STEPS
		break
		;;
	"SM_A202X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A202X
		COMMON_STEPS
		break
		;;
	"SM_A305X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A305X
		COMMON_STEPS
		break
		;;
	"SM_A307X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A307X
		COMMON_STEPS
		break
		;;
	"SM_A405X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_A405X
		COMMON_STEPS
		break
		;;
	"SM_M205X")
		echo " "
		echo "Android versions available: "
		echo " "
		OS_MENU
		echo " "
		SM_M205X
		COMMON_STEPS
		break
		;;
	"Exit")
		echo " Exiting build script.."
		sleep 2
		exit
		;;
	*)
		echo Invalid option.
		;;
	esac
done
