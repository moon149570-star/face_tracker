set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

set(TOOLCHAIN_ROOT
    "/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1"
)

set(CMAKE_C_COMPILER
    "${TOOLCHAIN_ROOT}/bin/loongarch64-linux-gnu-gcc"
)

set(CMAKE_CXX_COMPILER
    "${TOOLCHAIN_ROOT}/bin/loongarch64-linux-gnu-g++"
)

set(CMAKE_SYSROOT
    "${TOOLCHAIN_ROOT}/loongarch64-linux-gnu/sysroot"
)

set(CMAKE_FIND_ROOT_PATH
    "${CMAKE_SYSROOT}"
    "/opt/opencv_4_10_build"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
