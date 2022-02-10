# Eureka Kernel Build Script

## Host compatibility

Eureka Kernel build script uses Vortex Clang 14 toolchain which has been built on Ubuntu 21.10 and compatibility of that toolchain on other linux distros cannont be guaranteed. 
If you wish to use our build script and you are on another distro, then you can use Proton Clang 13 toolchain. Before executing the script, execute the following in the kernel source folder u cloned:

rm -rf toolchain

git clone --depth=1 https://github.com/kdrag0n/proton-clang.git toolchain/

## Build script executing methods:
Please add your telegram variables at line 32 in the script before executing it. In case you do not want to use it. Change "export BOT_TOKEN=0" to "export BOT_TOKEN=1".

### Option 1:
Run "./build.sh" to run the script in manual mode. This means you can choose what to build. You will get only anykernel flashlable zip with default DTB in this option.

### Option 2:
Run the script in automatic mode. This will build all kernels and you will get each device's AROMA flashable zip with all custom DTBs included.
If you are on HMP branch, then:
run "./build.sh auto hmp"
else if on EMS branch, then:
run "./build.sh auto ems"

### Build AROMA flashable zip only for 1 device.
Well, we did not add support for that because we did not deem it necessary :)
However, you can edit the build script before executing it. In the build_all() function at line 722. Comment out the devices which you do not want to build.
For example, assume I want only A30 AROMA zip. I will edit the script as follows:

SM_A105X         -> # SM_A105X

COMMON_STEPS     -> # COMMON_STEPS

SM_A205X         -> # SM_A205X

COMMON_STEPS     -> # COMMON_STEPS

SM_A202X         -> # SM_A202X

COMMON_STEPS     -> # COMMON_STEPS

SM_A305X         -> SM_A305X

COMMON_STEPS     -> COMMON_STEPS

SM_A307X         -> # SM_A307X

COMMON_STEPS     -> # COMMON_STEPS

SM_A405X         -> # SM_A405X

COMMON_STEPS     -> # COMMON_STEPS

### Option 3:
Run the script in DTB generator mode. This useful if you made some changes to Exynos7885 dts files. Note, when using option 1 or 2, default DTB is always regenerated from source. Only custom DTBs are prebuilt so as not to waste time rebuilding them each time.

To generate only default DTB (Please note that sound driver compatibility and selinux type depends on DTB. Please ensure that you edited what you need before):
run "./build.sh dtb"

To regenerate all prebuilt custom DTBs in the source (Sound driver compatibility and selinux type are already taken care by the script!):
run "./build.sh dtb auto"
