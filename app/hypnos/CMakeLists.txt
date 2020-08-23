# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Add the pinetime board
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

# Config files
IF("$ENV{LOGGING}" STREQUAL "off")
  message(STATUS "LOGGING will be disabled.")
  set(CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf)
ELSE()
  message(STATUS "LOGGING will be enabled.")
  set(CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf ${CMAKE_CURRENT_LIST_DIR}/logging.conf)
ENDIF()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
include_directories(include)
project(hypnos)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Get current time
string(TIMESTAMP CURRENT_TIME %Y-%m-%dT%H:%M:%S)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCURRENT_TIME_OF_BUILD=\"${CURRENT_TIME}\"")
