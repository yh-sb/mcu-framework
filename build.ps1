$ErrorActionPreference = "Stop"

$build_type = "Debug"

& 'cmake' . -Bbuild -G Ninja -DCMAKE_BUILD_TYPE=Debug
& 'cmake' --build build -j
