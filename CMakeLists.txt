cmake_minimum_required(VERSION 3.17)
project(psx_cdrom_lle)

set(CMAKE_CXX_STANDARD 17)

add_executable(psx_cdrom_lle
        src/mc68hc05.cpp
        src/main.cpp)