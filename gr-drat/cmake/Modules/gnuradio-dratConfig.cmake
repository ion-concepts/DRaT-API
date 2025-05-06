find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_DRAT gnuradio-drat)

FIND_PATH(
    GR_DRAT_INCLUDE_DIRS
    NAMES gnuradio/drat/api.h
    HINTS $ENV{DRAT_DIR}/include
        ${PC_DRAT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_DRAT_LIBRARIES
    NAMES gnuradio-drat
    HINTS $ENV{DRAT_DIR}/lib
        ${PC_DRAT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-dratTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_DRAT DEFAULT_MSG GR_DRAT_LIBRARIES GR_DRAT_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_DRAT_LIBRARIES GR_DRAT_INCLUDE_DIRS)
