cmake_minimum_required(VERSION 3.20)

project(caudiora C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(RTLSDR REQUIRED IMPORTED_TARGET librtlsdr)

add_executable(caudiora
        src/main.c
        src/radio_interface.c
        src/acquisition.c
        src/processing.c
)

target_link_libraries(caudiora PRIVATE PkgConfig::RTLSDR m)