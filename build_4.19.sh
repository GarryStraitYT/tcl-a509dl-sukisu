#!/bin/bash

echo -e "\n[INFO]: BUILD STARTED..!\n"

#init submodules
git submodule init && git submodule update

export KERNEL_ROOT="$(pwd)"
export ARCH=arm64
#export KBUILD_BUILD_USER="@ravindu644"

# Function to detect OS and install dependencies
install_dependencies() {
    echo -e "\n[INFO]: Detecting OS and installing dependencies...\n"

    if command -v dnf &> /dev/null; then
        echo -e "[INFO]: Fedora/RHEL-based system detected, using dnf...\n"
        sudo dnf group install "c-development" "development-tools" && \
        sudo dnf install -y dtc lz4 xz zlib-devel java-latest-openjdk-devel python3 \
            p7zip p7zip-plugins android-tools erofs-utils \
            ncurses-devel libX11-devel readline-devel mesa-libGL-devel python3-markdown \
            libxml2 libxslt dos2unix kmod openssl elfutils-libelf-devel dwarves \
            openssl-devel libarchive zstd rsync openssl-devel-engine --skip-unavailable

    elif command -v apt &> /dev/null; then
        echo -e "[INFO]: Ubuntu/Debian-based system detected, using apt...\n"
        sudo apt update && sudo apt install -y git device-tree-compiler lz4 xz-utils zlib1g-dev openjdk-17-jdk gcc g++ python3 python-is-python3 p7zip-full android-sdk-libsparse-utils erofs-utils \
            default-jdk git gnupg flex bison gperf build-essential zip curl libc6-dev libncurses-dev libx11-dev libreadline-dev libgl1 libgl1-mesa-dev \
            python3 make sudo gcc g++ bc grep tofrodos python3-markdown libxml2-utils xsltproc zlib1g-dev python-is-python3 libc6-dev libtinfo6 \
            make repo cpio kmod openssl libelf-dev pahole libssl-dev libarchive-tools zstd rsync --fix-missing && wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb && sudo dpkg -i libtinfo5_6.3-2ubuntu0.1_amd64.deb

    else
        echo -e "[ERROR]: Neither dnf nor apt package manager found. Please install dependencies manually.\n"
        exit 1
    fi

    touch .requirements
}

# Install the requirements for building the kernel when running the script for the first time
if [ ! -f ".requirements" ]; then
    install_dependencies
fi
mkdir -p "${KERNEL_ROOT}/out" "${KERNEL_ROOT}/build" "/work/toolchains"

# Use the toolchains provisioned in the container.
export PATH="/work/toolchains/clang-r584948/bin:/work/toolchains/bangkok-cross/bin:${PATH}"
export LD_LIBRARY_PATH="/work/toolchains/clang-r584948/lib:${LD_LIBRARY_PATH}"

# Set cross-compile environment variables
export BUILD_CROSS_COMPILE="/work/toolchains/bangkok-cross/bin/aarch64-linux-gnu-"
export BUILD_CROSS_COMPILE_ARM32="/work/toolchains/bangkok-cross/bin/arm-linux-gnueabi-"
export BUILD_CC="/work/toolchains/clang-r584948/bin/clang"
# Build options for the kernel
export BUILD_OPTIONS=(
    -C "${KERNEL_ROOT}"
    O="${KERNEL_ROOT}/out"
    -j"$(nproc)"
    ARCH=arm64
    CC="${BUILD_CC}"
    CLANG_TRIPLE="aarch64-linux-gnu-"
    CROSS_COMPILE="${BUILD_CROSS_COMPILE}"
    CROSS_COMPILE_ARM32="${BUILD_CROSS_COMPILE_ARM32}"
    LD="ld.lld"
)

build_kernel(){
    # Make default configuration.
    # Replace 'your_defconfig' with the name of your kernel's defconfig
    make "${BUILD_OPTIONS[@]}" bangkok_NA_OM_defconfig

    # Configure the kernel (GUI)
    make "${BUILD_OPTIONS[@]}" menuconfig

    # Build the kernel
    make "${BUILD_OPTIONS[@]}" || exit 1

    # Copy the built kernel to the build directory
    cp "${KERNEL_ROOT}/out/arch/arm64/boot/Image" "${KERNEL_ROOT}/build"

    echo -e "\n[INFO]: BUILD FINISHED..!"
}
build_kernel
