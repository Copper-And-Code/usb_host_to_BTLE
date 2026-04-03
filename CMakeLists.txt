# Top-level CMakeLists.txt for ESP-IDF project
# USB Host to BLE Keyboard Bridge with VT100 terminal emulation

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(usb_host_to_btle)
