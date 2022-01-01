#!/bin/sudo bash
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
ONEUI3=0
GCC_ARM64_FILE=aarch64-linux-gnu-
GCC_ARM32_FILE=arm-linux-gnueabi-

# Export commands
export KBUILD_BUILD_USER=Eureka
export KBUILD_BUILD_HOST=Eureka.org
export ARCH=arm64
export CROSS_COMPILE=$(pwd)/toolchain/bin/$GCC_ARM64_FILE
export CROSS_COMPILE_ARM32=$(pwd)/toolchain/bin/$GCC_ARM32_FILE

# Get date and time
DATE=$(date +"%m-%d-%y")
BUILD_START=$(date +"%s")

######################### Colours ############################

ON_BLUE=`echo -e "\033[44m"`
RED=`echo -e "\033[1;31m"`
BLUE=`echo -e "\033[1;34m"`
GREEN=`echo -e "\033[1;32m"`
STD=`echo -e "\033[0m"`		# Clear colour


####################### Devices List #########################

SM_A105X() {

	CODENAME=A105
	DEFCONFIG=exynos7885-a10_"$SELINUX_STATUS"defconfig
}

SM_A205X() {
	CODENAME=A205
	DEFCONFIG=exynos7885-a20_"$SELINUX_STATUS"defconfig
}

SM_A202X() {
	CODENAME=A202
	DEFCONFIG=exynos7885-a20e_"$SELINUX_STATUS"defconfig
}

SM_A305X() {
	CODENAME=A305
	DEFCONFIG=exynos7885-a30_"$SELINUX_STATUS"defconfig
}

SM_A307X() {
	CODENAME=A307
	DEFCONFIG=exynos7885-a30s_"$SELINUX_STATUS"defconfig
}

SM_A405X() {
	CODENAME=A405
	DEFCONFIG=exynos7885-a40_"$SELINUX_STATUS"defconfig
}

SM_M205X() {
	CODENAME=M205
	DEFCONFIG=exynos7885-m20_"$SELINUX_STATUS"defconfig
}


######################## Android OS list #######################

android_qrs="Support for OneUI 2, AOSP 10 (Q), 11 (R) & 12 (S)"
android_oneui3="Support for OneUI 3 only"


################### Executable functions #######################

CLANG_CLEAN() {
	echo " ${ON_BLUE}Cleaning kernel source ${STD}"
	echo " "

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
}

TOOLCHAIN() {
	if [ -e "toolchain/bin/clang-13" ]; then
		{
			echo " "
			echo " ${GREEN}Using Clang 13 as compiler ${STD}"
			echo " "
			GCC_ARM64_FILE=aarch64-linux-gnu-
			GCC_ARM32_FILE=arm-linux-gnueabi-
			echo " "
		}
	else
		echo " "
		echo " ${RED}WARNING: Correct toolchain could not be found! Downloading latest toolchain. ${STD}"
		echo " "
                git clone --depth=1 https://github.com/kdrag0n/proton-clang.git toolchain/
		sleep 2
	fi
}

CLANG_BUILD() {
	export LOCALVERSION=-$VERSION
	export PLATFORM_VERSION=$AND_VER
	make O=out ARCH=arm64 ANDROID_MAJOR_VERSION=$ANDROID $DEFCONFIG > /dev/null
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
	if [ "${CODENAME}" == "A105" ]; then
		cp -f kernel_zip/dtbo/a10/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "A205" ]; then
		cp -f kernel_zip/dtbo/a20/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "A202" ]; then
		cp -f kernel_zip/dtbo/a20e/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "A305" ]; then
		cp -f kernel_zip/dtbo/a30/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "A307" ]; then
		cp -f kernel_zip/dtbo/a30s/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "A405" ]; then
		cp -f kernel_zip/dtbo/a40/dtbo.img kernel_zip/anykernel/dtbo.img
	elif [ "${CODENAME}" == "M205" ]; then
		if [ -e "kernel_zip/dtbo/m20/dtbo.img" ]; then
			cp -f kernel_zip/dtbo/m20/dtbo.img kernel_zip/anykernel/dtbo.img
		fi
	fi
}

ZIPPIFY() {
	# Make Eureka flashable zip

	if [ -e "arch/$ARCH/boot/Image" ]; then
		{
			echo -e " "
			echo -e " ${ON_BLUE}Building Eureka anykernel flashable zip ${STD}"
			echo -e " "

			# Copy Image, dtb.img and dtbo.img to anykernel directory
			cp -f arch/$ARCH/boot/Image kernel_zip/anykernel/Image
			if [ -e "arch/$ARCH/boot/dtb.img" ]; then
				if [ "${CODENAME}" == "M205" ]; then
					# Don't copy dtb to anykernel directory because it has never been tested.
					cp -f arch/$ARCH/boot/dtb.img kernel_zip/m20_dtb.img
				else
					cp -f arch/$ARCH/boot/dtb.img kernel_zip/anykernel/dtb.img
				fi
			fi
			PREBUILT_DTBO
			#cp -f arch/$ARCH/boot/dtbo.img kernel_zip/anykernel/dtbo.img

			# Go to anykernel directory
			cd kernel_zip/anykernel

			if [ -e "dtb.img" ]; then
				{
					if [ -e "dtbo.img" ]; then
						{
							zip -r9 $ZIPNAME META-INF tools anykernel.sh Image dtb.img dtbo.img version > /dev/null
						}
					else
						{
							mv anykernel.sh anykernel.sh.bak
							sed '58,61d' anykernel.sh.bak > anykernel.sh
							zip -r9 $ZIPNAME META-INF tools anykernel.sh Image dtb.img version > /dev/null
							rm -f anykernel.sh
							mv anykernel.sh.bak anykernel.sh
							chown a+x anykernel.sh
						}
					fi
				}
			else
				{
					mv anykernel.sh anykernel.sh.bak
					sed '53,61d' anykernel.sh.bak > anykernel.sh
					zip -r9 $ZIPNAME META-INF tools anykernel.sh Image version > /dev/null
					rm -f anykernel.sh
					mv anykernel.sh.bak anykernel.sh
				}
			fi

			chmod 0777 $ZIPNAME
			# Go back into kernel source directory
			cd ..
			sleep 1
			cd ..
			sleep 1
		}
	fi
}

PROCESSES() {
	# Allow user to choose how many cores to be taken by compiler
	echo " ${ON_BLUE}Your system has $CORES cores. ${STD}"
	echo " "
	read -p " ${GREEN}Please enter how many cores to be used by compiler (Leave blank to use all cores) : " cores
	if [ "${cores}" == "" ]; then
		echo " "
		echo " Using all $CORES cores for compilation. ${STD}"
		sleep 1
	else
		echo " "
		echo " Using $cores cores for compilation. ${STD}"
		CORES=$cores
		sleep 1
	fi
}

ENTER_VERSION() {
	# Enter kernel version for this build.
	REV="$(grep -Po 'Eureka R\K[^*]+' kernel_zip/anykernel/version)"
	echo " ${ON_BLUE}Current Kernel Version: $REV ${STD}"
	echo " "
	read -p " ${GREEN}Please type kernel version without 'R' (E.g: $REV) : " rev
	if [ "${rev}" == "" ]; then
		REV="$REV-$((RANDOM % 999))"
		echo " "
		echo " Using '$REV' as test version ${STD}"
	else
		REV=$rev
		echo " "
		echo " Version = $REV ${STD}"
	fi
	sleep 1
}

USER() {
	# Setup KBUILD_BUILD_USER

	username="$(who | sed 's/  .*//')"
	USER=${username^}
	echo " ${ON_BLUE}Current build_user is $USER ${STD}"
	echo " "
	read -p " ${GREEN}Please define build_user (E.g: $USER) : " user
	if [ "${user}" == "" ]; then
		export KBUILD_BUILD_USER=$USER
		echo " "
		echo " Using '$USER' as build_user ${STD}"
	else
		export KBUILD_BUILD_USER=$user
		USER=$user
		echo " "
		echo " build_user = $USER ${STD}"
	fi
	sleep 1
}

RENAME() {
	# Give proper name to kernel and zip name
	if [ "$ONEUI3" == "1" ]; then
		VERSION="Eureka_R"$REV"_"$CODENAME"_"$SELINUX_STATUS"OneUI3/4"
		ZIPNAME="Eureka_R"$REV"_"$CODENAME"_"$SELINUX_STATUS"OneUI3.zip"
	else
		VERSION="Eureka_R"$REV"_"$CODENAME"_"$SELINUX_STATUS"Q/R/S"
		ZIPNAME="Eureka_R"$REV"_"$CODENAME"_"$SELINUX_STATUS"Q_R_S.zip"
	fi
}

SELINUX() {
	echo " ${ON_BLUE}Choose which SElinux state you wish to have ${STD}"
	echo " ${BLUE}"
	echo "  1) Build Eureka with ENFORCING SElinux"
	echo " "
	echo "  2) Build Eureka with PERMISSIVE SElinux"
	echo " "
	echo "  [Leave empty to exit this script]"
	echo " ${STD}"
	read -n 1 -p " ${GREEN}Select your choice: " -s choice
	case ${choice} in
	1)
		{
			export SELINUX_B=enforcing
			export SELINUX_STATUS="$SELINUX_B"_
			echo " "
			echo " "
			echo " ${GREEN}Enforcing chosen. Good choice :) ${STD}"
			sleep 1
		}
		;;
	2)
		{
			export SELINUX_B=permissive
			export SELINUX_STATUS="$SELINUX_B"_
			echo " "
			echo " "
			echo " ${GREEN}Permissive chosen. Use with caution! ${STD}"
			sleep 1
		}
		;;
	*)
		{
			echo " "
			echo " "
			echo " ${RED}Invalid choice entered. Exiting... ${STD}"
			sleep 1
			exit 1
		}
		;;
	esac
	sleep 1
}

WIREGUARD_PERM() {
	# Since wireguard checks for update during compilation, its group will change from $username to root.
	# So change it back to default user group.
	chown -R $username $(pwd)/net/wireguard
}

DISPLAY_ELAPSED_TIME() {
	# Find out how much time build has taken
	BUILD_END=$(date +"%s")
	DIFF=$(($BUILD_END - $BUILD_START))

	BUILD_SUCCESS=$?
	if [ $BUILD_SUCCESS != 0 ]; then
		echo " ${RED}Error: Build failed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds $reset ${STD}"
		exit
	fi

	echo -e " ${GREEN}Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds $reset ${STD}"
	sleep 1
}

COMMON_STEPS() {
	clear
	echo " ${ON_BLUE}Starting compilation ${STD}"
	echo " "
	echo " ${GREEN}Defconfig loaded: $DEFCONFIG ${STD}"
	RENAME
	sleep 1
	echo " ${BLUE}"
	CLANG_BUILD
	echo " ${STD}"
	sleep 1
	cp -f out/arch/$ARCH/boot/Image arch/$ARCH/boot/Image
	cp -f out/arch/$ARCH/boot/dtb.img arch/$ARCH/boot/dtb.img
	cp -f out/arch/$ARCH/boot/dtbo.img arch/$ARCH/boot/dtbo.img
	ZIPPIFY
	sleep 1
	echo " "
	CLANG_CLEAN
	sleep 1
	echo " "
	WIREGUARD_PERM
	DISPLAY_ELAPSED_TIME
	echo " ${BLUE}"
	echo " ___________                     __            "
	echo " \_   _____/__ _________   ____ |  | _______   "
	echo "  |    __)_|  |  \_  __ \_/ __ \|  |/ /\__  \  "
	echo "  |        \  |  /|  | \/\  ___/|    <  / __ \_"
	echo " /_______  /____/ |__|    \___  >__|_ \(____  /"
	echo "         \/                   \/     \/     \/ "
	echo " "
	echo " $CODENAME kernel R"$REV" for $ANDROID_VAR ${STD}"
	echo " "
}

OS_MENU() {
	# Give the choice to choose Android Version
	echo " ${ON_BLUE}Android Versions Available: ${STD}"
	echo " ${GREEN}"
	echo " 1) $android_qrs"
	echo " "
	echo " 2) $android_oneui3"
	echo " "
	read -n 1 -p " Please select your Android Version: ${STD}" -s menuos
	case $menuos in
	1)
		{
			echo " "
			ANDROID_VAR="Android 10 (Q) / 11 (R) / 12 (S)"
			echo " "
			echo "${GREEN} $ANDROID_VAR chosen as Android Major Version ${STD}"
			ANDROID=r
			AND_VER=11
			sleep 2
			echo " "
		}
		;;
	2)
		{
			echo " "
			ANDROID_VAR="Android 11 (OneUI 3)"
			echo " "
			echo "${GREEN} $ANDROID_VAR chosen as Android Major Version ${STD}"
			ANDROID=r
			AND_VER=11
			ONEUI3=1
			sleep 2
			echo " "
		}
		;;
	*)
		{
			echo " "
			echo " ${RED}Exiting build script... ${STD}"
			sleep 2
			echo " "
			exit
		}
		;;
	esac
	sleep 1
}


###################### Script starts here #######################

clear
TOOLCHAIN
clear
CLANG_CLEAN
sleep 1
clear
PROCESSES
clear
ENTER_VERSION
clear
USER
clear
SELINUX
clear
echo "${BLUE}******************************************************"
echo "*                                                    *"
echo "*             $PROJECT_NAME Build Script             *"
echo "*                  Developer: Chatur                 *"
echo "*            Co-Developers: Gabriel, Royna           *"
echo "*                                                    *"
echo "******************************************************"
echo " Some informations about parameters set:		"
echo -e "    > Architecture: $ARCH				"
echo "    > Jobs: $CORES					"
echo "    > Revision for this build: R$REV			"
echo "    > SElinux Status: $SELINUX_B			"
echo "    > Build user: $KBUILD_BUILD_USER			"
echo "    > Build machine: $KBUILD_BUILD_HOST		"
echo "    > ARM64 Toolchain path exported			"
echo "    > ARM32 Toolchain path exported			"
echo -e "*****************************************************"
echo " "

echo "${STD} ${ON_BLUE}Devices available: ${STD}"
PS3='
 Please select your device: '
echo " ${GREEN}"
menuoptions=("SM_A105X" "SM_A205X" "SM_A202X" "SM_A305X" "SM_A307X" "SM_A405X" "SM_M205X" "Exit")
select menuoptions in "${menuoptions[@]}"; do
	case $menuoptions in
	"SM_A105X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A105X
		COMMON_STEPS
		break
		;;
	"SM_A205X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A205X
		COMMON_STEPS
		break
		;;
	"SM_A202X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A202X
		COMMON_STEPS
		break
		;;
	"SM_A305X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A305X
		COMMON_STEPS
		break
		;;
	"SM_A307X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A307X
		COMMON_STEPS
		break
		;;
	"SM_A405X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_A405X
		COMMON_STEPS
		break
		;;
	"SM_M205X")
		echo " ${STD}"
		OS_MENU
		echo " "
		SM_M205X
		COMMON_STEPS
		break
		;;
	"Exit")
		echo " ${RED}Exiting build script... ${STD}"
		sleep 2
		exit
		;;
	*)
		echo " "
		echo " ${RED}Invalid option. Try again. ${STD}"
		;;
	esac
done
