# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/../pinetime/cmake/boilerplate.cmake)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
include_directories(include)
project(hypnos)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Get current time
execute_process(
  COMMAND date --iso-8601=seconds # e.g. 2020-04-04T14:17:34+02:00
  OUTPUT_VARIABLE CURRENT_TIME
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCURRENT_TIME_OF_BUILD=\"${CURRENT_TIME}\"")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include clock.h")
