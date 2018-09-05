cmake_minimum_required(VERSION 2.8.8)

include(${CMAKE_SOURCE_DIR}/cmake/fx3.cmake)

project(adsdrfx3)

#fx3_enable_cxx()
#fx3_enable_stdcxx_libs()

set(FX3_CORE_C_SRC "${PROJECT_SOURCE_DIR}/src/cyfxtx.c")

set(COMMON_FLAGS "-fmessage-length=0 -ffunction-sections -fdata-sections -fshort-wchar -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-unwind-tables -std=c++11 ${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} -Wwrite-strings -std=c99")

file(GLOB_RECURSE ADSDRFX3_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.h
    ${PROJECT_SOURCE_DIR}/src/*.c
)

fx3_add_target(adsdrfx3
    ${ADSDRFX3_SRC_FILES}
)