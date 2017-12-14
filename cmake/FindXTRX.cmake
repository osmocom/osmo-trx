# - Find the XTRX library
#
# Usage:
#   find_package(XTRX [REQUIRED] [QUIET] )
#     
# It sets the following variables:
#   XTRX_FOUND               ... true if XTRX is found on the system
#   XTRX_LIBRARIES           ... full path to XTRX library
#   XTRX_INCLUDES            ... XTRX include directory
#
# The following variables will be checked by the function
#   XTRX_USE_STATIC_LIBS    ... if true, only static libraries are found
#   XTRX_ROOT               ... if set, the libraries are exclusively searched
#                               under this path
#   XTRX_LIBRARY            ... XTRX library to use
#   XTRX_INCLUDE_DIR        ... XTRX include directory
#
#If environment variable XTRXDIR is specified, it has same effect as XTRX_ROOT
if( NOT XTRX_ROOT AND ENV{XTRXDIR} )
  set( XTRX_ROOT $ENV{XTRXDIR} )
endif()
# Check if we can use PkgConfig
find_package(PkgConfig)
#Determine from PKG
if( PKG_CONFIG_FOUND AND NOT XTRX_ROOT )
  pkg_check_modules( PKG_XTRX QUIET "libxtrx" )
endif()
#Check whether to search static or dynamic libs
set( CMAKE_FIND_LIBRARY_SUFFIXES_SAV ${CMAKE_FIND_LIBRARY_SUFFIXES} )
if( ${XTRX_USE_STATIC_LIBS} )
  set( CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX} )
else()
  set( CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_SHARED_LIBRARY_SUFFIX} )
endif()
if( XTRX_ROOT )
  #find libs
  find_library(
    XTRX_LIB
    NAMES "xtrx"
    PATHS ${XTRX_ROOT}
    PATH_SUFFIXES "lib" "lib64"
    NO_DEFAULT_PATH
  )
  #find includes
  find_path(
    XTRX_INCLUDES
    NAMES "xtrx_api.h"
    PATHS ${XTRX_ROOT}
    PATH_SUFFIXES "include"
    NO_DEFAULT_PATH
  )
else()
  find_library(
    XTRX_LIB
    NAMES "xtrx"
    PATHS ${PKG_XTRX_LIBRARY_DIRS} ${LIB_INSTALL_DIR}
  )
  find_path(
    XTRX_INCLUDES
    NAMES "xtrx_api.h"
    PATHS ${PKG_XTRX_INCLUDE_DIRS} ${INCLUDE_INSTALL_DIR}
  )
endif( XTRX_ROOT )
set(XTRX_LIBRARIES ${XTRX_LIB})
set( CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES_SAV} )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XTRX DEFAULT_MSG
                                  XTRX_INCLUDES XTRX_LIBRARIES)
mark_as_advanced(XTRX_INCLUDES XTRX_LIBRARIES XTRX_LIB XTRXF_LIB XTRXL_LIB)
