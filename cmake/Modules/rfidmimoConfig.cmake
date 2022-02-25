INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_RFIDMIMO rfidmimo)

FIND_PATH(
    RFIDMIMO_INCLUDE_DIRS
    NAMES rfidmimo/api.h
    HINTS $ENV{RFIDMIMO_DIR}/include
        ${PC_RFIDMIMO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    RFIDMIMO_LIBRARIES
    NAMES gnuradio-rfidmimo
    HINTS $ENV{RFIDMIMO_DIR}/lib
        ${PC_RFIDMIMO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/rfidmimoTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RFIDMIMO DEFAULT_MSG RFIDMIMO_LIBRARIES RFIDMIMO_INCLUDE_DIRS)
MARK_AS_ADVANCED(RFIDMIMO_LIBRARIES RFIDMIMO_INCLUDE_DIRS)
