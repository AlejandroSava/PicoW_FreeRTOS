# PicoW_FreeRTOS
This repository is to track my projects related to real time systems 😮‍💨

I have taken part of the structure from: 
https://github.com/PicoCPP/RPI-pico-FreeRTOS/tree/master

## Linux Setup 
sudo apt update
sudo apt install cmake gcc-arm-none-eabi ninja-build git
mkdir -p ~/pico
cd ~/pico
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
export PICO_SDK_PATH=~/pico/pico-sdk
echo "export PICO_SDK_PATH=~/pico/pico-sdk" >> ~/.bashrc
source ~/.bashrc
cd ~/pico
mkdir -p libs
cd libs
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
cd FreeRTOS-Kernel
git submodule update --init --recursive
export FREERTOS_KERNEL_PATH=~/pico/libs/FreeRTOS-Kernel
echo "export FREERTOS_KERNEL_PATH=~/pico/libs/FreeRTOS-Kernel" >> ~/.bashrc
source ~/.bashrc
