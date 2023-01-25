#
#
#

#
# Options.
#

cmake_minimum_required(VERSION 3.20.0)

#
set(BOARD nrf52840dongle_nrf52840)
# set(BOARD xiao_ble)

set(uav_operator GBR-OP-ABCD12345678)
set(uav_id ZZZZH1234567)
set(self_id CAA\ UAS\ 7068)
# ODID_UATYPE_AEROPLANE
set(odid_uatype ODID_UATYPE_NONE)
# ODID_CATEGORY_EU_UNDECLARED
set(odid_cat ODID_CATEGORY_EU_SPECIFIC)

# National/regional variations.
set(id_japan 0)
set(id_usa   0)

# Experimental.
set(spektrum   0)
set(batt_volts 0)

set(CMAKE_C_FLAGS   "-Wall -Wno-parentheses -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "-Wall -Wno-parentheses -Wno-deprecated-declarations")

#
# Build.
#

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(remote_id_nrf52)

# Source files.

configure_file(src/remote_id.h.in ../src/remote_id.h)

set(c_sources src/opendroneid.c src/alt_unix_time.c)

set(cpp_sources src/main_nrf52.cpp src/gps2.cpp src/odid_bt.cpp)

if (${batt_volts})
  set(cpp_sources ${cpp_sources} src/battery.cpp)
endif()

if (${id_japan})
  set(cpp_sources ${cpp_sources} src/local.cpp)
endif()

if (${spektrum})
  set(cpp_sources ${cpp_sources} src/spektrum_i2c.cpp)
endif()

target_sources(app PRIVATE ${cpp_sources} ${c_sources})
