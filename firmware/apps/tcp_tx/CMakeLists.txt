cmake_minimum_required(VERSION 3.20.0)

# boards and the TSEMAC driver live in the workspace's module
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../module)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tcp_tx)

target_sources(app PRIVATE src/main.c)
