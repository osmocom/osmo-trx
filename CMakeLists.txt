cmake_minimum_required(VERSION 2.8)
project(osmo-trx C CXX)

set(CMAKE_CXX_STANDARD 11)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION git)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

include_directories(CommonLibs)
include_directories(GSM)

add_definitions(-Wall -g)

#set(BUILD_SHARED_LIBS ON)

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
@ONLY)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(TRANS_FULL_VERSION "Compile with all Multichannel/Resampler support" OFF)
#option(SQLITE_CONFIG      "Use config values from SQLite3 database" OFF)
set(SQLITE_CONFIG ON)

if(TRANS_FULL_VERSION)
    find_package(FFTW)
endif(TRANS_FULL_VERSION)

find_package(XTRX)

if(SQLITE_CONFIG)
    find_library(sqlite3 sqlite3)
else(SQLITE_CONFIG)
    add_definitions(-DNO_SQLITE_CONFIG)
endif(SQLITE_CONFIG)

include_directories(${CMAKE_CURRENT_BINARY_DIR})


add_subdirectory(CommonLibs)
add_subdirectory(GSM)
add_subdirectory(Transceiver52M)


