# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(my_ble_central)

# add_executable(my_ble_central my_ble_central.c hid_host_example.c)