# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(central_multilink_custom_nRF21540)

set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

