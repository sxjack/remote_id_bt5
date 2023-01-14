#
#
#

cmake_minimum_required(VERSION 3.20.0)

# set(BOARD nrf52840dongle_nrf52840)
set(BOARD xiao_ble)

set(cmake_c_flags   "-Wall -Wno-parentheses -Wno-deprecated-declarations")
set(cmake_cpp_flags "-Wall -Wno-parentheses -Wno-deprecated-declarations")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(remote_id_nrf52)

set(c_sources   src/opendroneid.c src/alt_unix_time.c)
set(cpp_sources src/main_nrf52.cpp src/gps2.cpp src/odid_bt.cpp)
# set(cpp_sources ${cpp_sources} src/spektrum_i2c.cpp)

target_sources(app PRIVATE ${cpp_sources} ${c_sources})

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
