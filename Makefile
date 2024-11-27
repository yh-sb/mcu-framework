# This makefile is used to shorten build commands.
# You can build the project without it, just calling the CMake manually.

BUILD_DIR ?= build
BUILD_TYPE ?= Debug

.PHONY: flash erase reset clean esp32s3 rp2040 stm32f0 stm32f1 stm32f4

all:
	cmake . -B$(BUILD_DIR) -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	cmake --build $(BUILD_DIR) -j

flash erase reset debug:
	cmake --build $(BUILD_DIR) --target $@

clean:
	@cmake -E rm -rf $(BUILD_DIR)

# Make the ESP32 S3 example the default one
esp32s3:
ifeq ($(OS),Windows_NT)
	xcopy /y examples\esp32s3\CMakeLists.txt .
	xcopy /y examples\esp32s3\wi-fi-example.cpp main.cpp
else
	cp examples/esp32s3/CMakeLists.txt .
	cp examples/esp32s3/wi-fi-example.cpp main.cpp
endif

# Make the rp2040 example the default one
rp2040:
ifeq ($(OS),Windows_NT)
	xcopy /y examples\rp2040\CMakeLists.txt .
	xcopy /y examples\rp2040\blink-example.cpp main.cpp
else
	cp examples/rp2040/CMakeLists.txt .
	cp examples/rp2040/blink-example.cpp main.cpp
endif

# Make the stm32f0 example the default one
stm32f0:
ifeq ($(OS),Windows_NT)
	xcopy /y examples\stm32f0\CMakeLists.txt .
	xcopy /y examples\stm32f0\blink-example.cpp main.cpp
else
	cp examples/stm32f0/CMakeLists.txt .
	cp examples/stm32f0/blink-example.cpp main.cpp
endif

# Make the stm32f1 example the default one
stm32f1:
ifeq ($(OS),Windows_NT)
	xcopy /y examples\stm32f1\CMakeLists.txt .
	xcopy /y examples\stm32f1\blink-example.cpp main.cpp
else
	cp examples/stm32f1/CMakeLists.txt .
	cp examples/stm32f1/blink-example.cpp main.cpp
endif

# Make the stm32f4 example the default one
stm32f4:
ifeq ($(OS),Windows_NT)
	xcopy /y examples\stm32f4\CMakeLists.txt .
	xcopy /y examples\stm32f4\blink-example.cpp main.cpp
else
	cp examples/stm32f4/CMakeLists.txt .
	cp examples/stm32f4/blink-example.cpp main.cpp
endif
