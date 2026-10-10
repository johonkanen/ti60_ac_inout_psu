cmake_minimum_required(VERSION 3.20.0)

# board ti60_psu and the register bus library live in the workspace's module
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../module)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(psu_test)

target_sources(app PRIVATE src/main.c)
