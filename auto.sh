#!/bin/sudo bash
#
# Custom build script for Eureka kernels by Chatur27, Gabriel2392 and roynatech2544 @Github - 2022
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

# Export Telegram variables
CHAT_ID=-1001525986158
BOT_TOKEN=5068662666:AAGZPHd5Gj6eOrJS2QyR0MzHmW03Ngq41vk

export KBUILD_BUILD_USER=Eureka
export KBUILD_BUILD_HOST=Eureka.org
export ARCH=arm64
export ONEUI3=1
export REV=9.5

SM_A105X() {
	CODENAME=A105
	DEFCONFIG=exynos7885-a10_defconfig
}

SM_A205X() {
	CODENAME=A205
	DEFCONFIG=exynos7885-a20_defconfig
}

SM_A202X() {
	CODENAME=A202
	DEFCONFIG=exynos7885-a20e_defconfig
}

SM_A305X() {
	CODENAME=A305
	DEFCONFIG=exynos7885-a30_defconfig
}

SM_A307X() {
	CODENAME=A307
	DEFCONFIG=exynos7885-a30s_defconfig
}

SM_A405X() {
	CODENAME=A405
	DEFCONFIG=exynos7885-a40_defconfig
}

CLEAN() {
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
	if [ ! -e "toolchain/bin/clang" ]; then
		{
            echo " "
			echo " ${RED}WARNING: Correct toolchain could not be found! Downloading latest Clang toolchain. ${STD}"
			echo " "
			#git clone --depth=1 https://github.com/kdrag0n/proton-clang.git toolchain/
			git clone --depth=1 https://github.com/vijaymalav564/vortex-clang.git toolchain/
		}
    fi
}

CLANG_BUILD() {
	export LOCALVERSION=-$VERSION
	make O=out ARCH=arm64 $DEFCONFIG > /dev/null
	PATH="`pwd`/toolchain/bin:${PATH}" \
		make -j`nproc` O=out \
		LLVM=1 \
		CC=clang \
		CROSS_COMPILE=aarch64-linux-gnu-
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
                cp -f arch/$ARCH/boot/dtb.img kernel_zip/anykernel/dtb.img
			fi

			# Comment & uncomment here to use DTBO.img from source
			PREBUILT_DTBO
			#cp -f arch/$ARCH/boot/dtbo.img kernel_zip/anykernel/dtbo.img

			# Go to anykernel directory
			cd kernel_zip/anykernel

			# Prepare kernel zip & update files for AROMA too
			cp -f tools/changelog.txt ../aroma/META-INF/com/google/android/aroma/
			mv tools/espectrum.zip ./
			mv anykernel.sh anykernel.sh.bak
			sed '53,79d' anykernel.sh.bak > anykernel.sh
			if [ "$ONEUI3" == "1" ]; then
				SPECIAL_ZIP_NAME=$CODENAME"_oneui3.zip"
			else
				SPECIAL_ZIP_NAME=$CODENAME"_aosp.zip"
			fi
			zip -r9 $SPECIAL_ZIP_NAME META-INF tools anykernel.sh Image > /dev/null
			chmod 0777 $SPECIAL_ZIP_NAME
			mv $SPECIAL_ZIP_NAME ../aroma/
			mv espectrum.zip tools/
			rm -f anykernel.sh
			mv anykernel.sh.bak anykernel.sh
			chmod a+x anykernel.sh
			# Go back into kernel source directory
			cd ../..
			sleep 1
		}
	fi
}


AROMA() {

	PREBUILT_DTBO
	cd kernel_zip

	# Copy dtbo from anykernel folder
	if [ ! -e "aroma/dtbo" ]; then
		mkdir aroma/dtbo
		chmod 0777 aroma/dtbo
	fi

	cd aroma
	mv $CODENAME"_oneui3.zip" kernel/oneui3.zip
	mv $CODENAME"_aosp.zip" kernel/aosp.zip
	zip -r9 $AROMAZIPNAME META-INF dtb dtbo kernel spectrum tools > /dev/nulll
	rm -f kernel/oneui3.zip
	rm -f kernel/aosp.zip
	rm -rf dtbo
	cd ../..
}


TELEGRAM_UPLOAD() {
	# Telegram functions.
	function tg_sendText() {
		curl -s "https://api.telegram.org/bot$BOT_TOKEN/sendMessage" \
		-d "parse_mode=html" \
		-d text="${1}" \
		-d chat_id=$CHAT_ID \
		-d "disable_web_page_preview=true"
	}
	function tg_sendFile() {
		curl -s "https://api.telegram.org/bot$BOT_TOKEN/sendDocument" \
		-F parse_mode=markdown \
		-F chat_id=$CHAT_ID \
		-F document=@${1} \
		-F "caption=$POST_CAPTION"
	}
		for files in ./kernel_zip/aroma/*.zip; do
			MODEL="$(echo "$files" | grep -Po $REV'_\K[^*_]+')"
			POST_CAPTION="Eureka R$REV for $MODEL (AROMA)"	
			tg_sendFile "$files" > /dev/null
			sleep 2
		done

}

RENAME() {
	# Give proper name to kernel and zip name
    	VERSION="Eureka_R"$REV"_"$CODENAME"_R_S"
	AROMAZIPNAME="Eureka_R"$REV"_"$CODENAME"_AROMA-"$SCHEDULER".zip"
}

COMMON_STEPS() {
	RENAME
	CLANG_BUILD
	cp -f out/arch/$ARCH/boot/Image arch/$ARCH/boot/Image
	cp -f out/arch/$ARCH/boot/dtb.img arch/$ARCH/boot/dtb.img
	cp -f out/arch/$ARCH/boot/dtbo.img arch/$ARCH/boot/dtbo.img
	ZIPPIFY
	CLEAN
}

if [ "$1" == "hmp" ]; then
    export SCHEDULER=HMP
elif [ "$1" == "ems" ]; then
    export SCHEDULER=EMS
else
    echo " ${RED}Missing first argument!"
    exit
fi

if [ "$2" == "oneui" ]; then
    export ONEUI3=1
elif [ "$2" == "aosp" ]; then
    export ONEUI3=0
elif [ "$2" == "zip" ]; then
	SM_A105X
	RENAME
	AROMA
	SM_A205X
	RENAME
	AROMA
	SM_A202X
	RENAME
	AROMA
	SM_A305X
	RENAME
	AROMA
	SM_A405X
	RENAME
	AROMA
	TELEGRAM_UPLOAD
	exit
else
    echo " ${RED}Missing second argument!"
    exit
fi

SM_A105X
COMMON_STEPS
SM_A205X
COMMON_STEPS
SM_A202X
COMMON_STEPS
SM_A305X
COMMON_STEPS
#SM_A307X
#COMMON_STEPS
SM_A405X
COMMON_STEPS
