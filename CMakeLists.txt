cmake_minimum_required(VERSION 3.17)
project(psx_cdrom_lle)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wimplicit-fallthrough
        -Wno-unused-variable
        -Wno-gnu-anonymous-struct)

add_executable(psx_cdrom_lle
        src/mc68hc05.cpp
        src/main.cpp)