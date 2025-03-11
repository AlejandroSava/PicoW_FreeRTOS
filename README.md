# 🚀 PicoW FreeRTOS
@Alejandro Salinas V.
---

Welcome to my **PicoW FreeRTOS** repository! 🎯 This repo is dedicated to tracking my projects related to **real-time systems**. 🕒⚡
-> Raspberry Pico W and FreeRTOS

> 💡 *Inspired by:* [RPI-pico-FreeRTOS](https://github.com/PicoCPP/RPI-pico-FreeRTOS/tree/master)

---

## 🛠️ Linux Setup

To get started, install the required dependencies and set up your development environment:

### **1️⃣ Install Dependencies**
```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi ninja-build git
```

### **2️⃣ Set Up the Raspberry Pi Pico SDK**
```bash
mkdir -p ~/pico
cd ~/pico
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
```

### **3️⃣ Export the Pico SDK Path**
```bash
export PICO_SDK_PATH=~/pico/pico-sdk
echo "export PICO_SDK_PATH=~/pico/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

### **4️⃣ Set Up FreeRTOS Kernel**
```bash
cd ~/pico
mkdir -p libs
cd libs
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
cd FreeRTOS-Kernel
git submodule update --init --recursive
```

### **5️⃣ Export the FreeRTOS Kernel Path**
```bash
export FREERTOS_KERNEL_PATH=~/pico/libs/FreeRTOS-Kernel
echo "export FREERTOS_KERNEL_PATH=~/pico/libs/FreeRTOS-Kernel" >> ~/.bashrc
source ~/.bashrc
```
### **6️⃣ Build your project**
```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```


---

## 🎯 Next Steps

- ✅ **Compile a FreeRTOS example** for Pico W.
- ✅ **Experiment with tasks, semaphores, and queues**.
- ✅ **Integrate networking and peripherals**.

🚀 Happy coding! 💡✨
