# - Try to find the libairspy library
# Once done this defines
#
#  LIBAIRSPYHF_FOUND - system has libairspy
#  LIBAIRSPYHF_INCLUDE_DIR - the libairspy include directory
#  LIBAIRSPYHF_LIBRARIES - Link these to use libairspy

# Copyright (c) 2013  Benjamin Vernoux
#


if (LIBAIRSPYHF_INCLUDE_DIR AND LIBAIRSPYHF_LIBRARIES)

  # in cache already
  set(LIBAIRSPYHF_FOUND TRUE)

else (LIBAIRSPYHF_INCLUDE_DIR AND LIBAIRSPYHF_LIBRARIES)
  IF (NOT WIN32)
    # use pkg-config to get the directories and then use these values
    # in the FIND_PATH() and FIND_LIBRARY() calls
    find_package(PkgConfig)
    pkg_check_modules(PC_LIBAIRSPYHF QUIET libairspyhf)
    if(NOT PC_LIBAIRSPYHF_FOUND)
      pkg_check_modules(PC_LIBAIRSPYHF QUIET airspyhf)
    endif()
  ENDIF(NOT WIN32)

  FIND_PATH(LIBAIRSPYHF_INCLUDE_DIR
    NAMES airspyhf.h
    HINTS $ENV{LIBAIRSPYHF_DIR}/include ${PC_LIBAIRSPYHF_INCLUDEDIR}
    PATHS /usr/local/include/libairspyhf /usr/include/libairspyhf /usr/local/include /usr/include /usr/include/airspyhf /usr/local/include/airspyhf
    ${CMAKE_SOURCE_DIR}/../libairspyhf/src
    /opt/local/include/libairspyhf
    /opt/local/include/airspyhf
    /opt/homebrew/include/libairspyhf
    /opt/homebrew/include/airspyhf
    ${LIBAIRSPYHF_INCLUDE_DIR}
  )

  set(libairspyhf_library_names airspyhf)

  FIND_LIBRARY(LIBAIRSPYHF_LIBRARIES
    NAMES ${libairspyhf_library_names}
    HINTS $ENV{LIBAIRSPYHF_DIR}/lib ${PC_LIBAIRSPYHF_LIBDIR}
    PATHS /usr/local/lib /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu /opt/local/lib /opt/homebrew/lib ${PC_LIBAIRSPYHF_LIBDIR} ${PC_LIBAIRSPYHF_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/../libairspyhf/src
  )

  if(LIBAIRSPYHF_INCLUDE_DIR)
    set(CMAKE_REQUIRED_INCLUDES ${LIBAIRSPYHF_INCLUDE_DIR})
  endif()

  if(LIBAIRSPYHF_LIBRARIES)
    set(CMAKE_REQUIRED_LIBRARIES ${LIBAIRSPYHF_LIBRARIES})
  endif()

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBAIRSPYHF DEFAULT_MSG LIBAIRSPYHF_LIBRARIES LIBAIRSPYHF_INCLUDE_DIR)

  MARK_AS_ADVANCED(LIBAIRSPYHF_INCLUDE_DIR LIBAIRSPYHF_LIBRARIES)

endif (LIBAIRSPYHF_INCLUDE_DIR AND LIBAIRSPYHF_LIBRARIES)
