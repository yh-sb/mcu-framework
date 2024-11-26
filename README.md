# mcu-cpp

[![Build](https://github.com/yh-sb/mcu-cpp/actions/workflows/build.yml/badge.svg)](https://github.com/yh-sb/mcu-cpp/actions/workflows/build.yml)

One C++ project for different microcontrollers types

See [examples](examples/).

## How to build
```bash
git clone --recursive https://github.com/yh-sb/mcu-cpp.git
cd mcu-cpp

make
```

## Requirements
* [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* [CMake](https://cmake.org/download)
* [Ninja](https://ninja-build.org)
* [JLink](https://www.segger.com/downloads/jlink) for flashing and debugging

## Requirements for ESP32 project
* GNU Xtensa esp-elf toolchain from [ESP-IDF v5.3.1 package](https://dl.espressif.com/dl/esp-idf)
* [Python](https://www.python.org/downloads)
* Install python dependencies:
    ```sh
    python -m pip install --upgrade pip
    python third_party/esp-idf/tools/idf_tools.py install-python-env
    python -m pip install -r third_party/esp-idf/tools/requirements/requirements.core.txt
    ```
