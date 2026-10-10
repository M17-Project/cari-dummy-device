cmake_minimum_required(VERSION 3.16)
project(cari-dummy-device VERSION 2.1.0 LANGUAGES C)

# CARI 2.1 slave emulating the M17 SX1255 RRU, built on libcari.
#
# The RRU core (rru.c, sx1255.c, ring.c) is hardware independent; the
# backend (hw_dummy.c, hw_rpi.c) is selected at run time with --hw.

option(RRU_HW_RPI "Build the Raspberry Pi backend skeleton (Linux spidev/GPIO)" ON)
option(RRU_BUILD_TESTS "Build tests" ON)
option(RRU_SANITIZE "Build with address/undefined sanitizers" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
if(RRU_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
    set(CARI_SANITIZE ON CACHE BOOL "" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(FindLibcari)
find_package(Threads REQUIRED)

# Shared RRU core + backends, reusable by the real RRU slave
add_library(rru_core STATIC
    src/rru.c
    src/sx1255.c
    src/ring.c
    src/hw_dummy.c)
if(RRU_HW_RPI)
    target_sources(rru_core PRIVATE src/hw_rpi.c)
    target_compile_definitions(rru_core PUBLIC RRU_HAVE_HW_RPI)
endif()
target_include_directories(rru_core PUBLIC src)
target_compile_definitions(rru_core PRIVATE _GNU_SOURCE)
target_compile_options(rru_core PRIVATE -Wall -Wextra -Wshadow)
target_link_libraries(rru_core PUBLIC cari::cari Threads::Threads m)

add_executable(cari-dummy-device src/main.c)
target_compile_definitions(cari-dummy-device PRIVATE _GNU_SOURCE)
target_compile_options(cari-dummy-device PRIVATE -Wall -Wextra)
target_link_libraries(cari-dummy-device PRIVATE rru_core)

include(GNUInstallDirs)
install(TARGETS cari-dummy-device RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(RRU_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
