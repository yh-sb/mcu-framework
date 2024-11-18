# mcu-cpp

[![Build](https://github.com/yh-sb/mcu-cpp/actions/workflows/build.yml/badge.svg)](https://github.com/yh-sb/mcu-cpp/actions/workflows/build.yml)

One C++ project for different microcontrollers types

See [examples](examples/).

## How to build
```bash
git clone --recursive https://github.com/yh-sb/mcu-cpp.git

# In Windows:
.\build.ps1

# In Linux:
./build.sh
```

## Requirements
* [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* [CMake](https://cmake.org/download)
* [Ninja](https://ninja-build.org)
* [JLink](https://www.segger.com/downloads/jlink) for flashing and debugging
