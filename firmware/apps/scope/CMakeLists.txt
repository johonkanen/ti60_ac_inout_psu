cmake_minimum_required(VERSION 3.20.0)

# board ti60_evm_eth and the TSEMAC/MDIO drivers live in the workspace's module
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../module)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scope)

target_sources(app PRIVATE src/main.c src/scope.c)

# the commit built (git_version.h for main.c's LCD line), looked at on every build
set(GIT_VERSION_DIR ${CMAKE_CURRENT_BINARY_DIR}/git_version)
add_custom_target(git_version
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${GIT_VERSION_DIR}/git_version.h
          -P ${CMAKE_CURRENT_SOURCE_DIR}/git_version.cmake
  BYPRODUCTS ${GIT_VERSION_DIR}/git_version.h)
add_dependencies(app git_version)
target_include_directories(app PRIVATE ${GIT_VERSION_DIR})
# the FPGA's scope_capture where the board has it, else the software stand-in
if(CONFIG_SCOPE_CAPTURE_HW)
  target_sources(app PRIVATE src/capture_hw.c)
  target_sources_ifdef(CONFIG_SCOPE_STREAM app PRIVATE src/stream.c)
else()
  target_sources(app PRIVATE src/capture_sim.c)
endif()

# HTTP resources of scope_service
zephyr_linker_sources(SECTIONS sections.ld)

# the web client in web/ is not in the image: upload_web.sh puts it in the board's flash
