# - Try to find the libusb-1.0 (USB1) library
# Once done this defines
#
#  USB1_FOUND - system has libusb-1.0
#  USB1_INCLUDE_DIR - the libusb include directory
#  USB1_LIBRARIES - Link these to use libusb
#
# Compatibility aliases are also provided:
#  LIBUSB_FOUND, LIBUSB_INCLUDE_DIR, LIBUSB_LIBRARIES

# Copyright (c) 2006, 2008  Laurent Montel, <montel@kde.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


if (USB1_INCLUDE_DIR AND USB1_LIBRARIES)

  # in cache already
  set(USB1_FOUND TRUE)

else (USB1_INCLUDE_DIR AND USB1_LIBRARIES)

  find_package(PkgConfig)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LIBUSB libusb-1.0)
  endif(PKG_CONFIG_FOUND)

  FIND_PATH(USB1_INCLUDE_DIR libusb.h
    PATHS ${PC_LIBUSB_INCLUDEDIR} ${PC_LIBUSB_INCLUDE_DIRS})

  FIND_LIBRARY(USB1_LIBRARIES NAMES usb-1.0
    PATHS ${PC_LIBUSB_LIBDIR} ${PC_LIBUSB_LIBRARY_DIRS})

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(USB1 DEFAULT_MSG USB1_LIBRARIES USB1_INCLUDE_DIR)

  MARK_AS_ADVANCED(USB1_INCLUDE_DIR USB1_LIBRARIES)

endif (USB1_INCLUDE_DIR AND USB1_LIBRARIES)

set(LIBUSB_FOUND ${USB1_FOUND})
set(LIBUSB_INCLUDE_DIR ${USB1_INCLUDE_DIR})
set(LIBUSB_LIBRARIES ${USB1_LIBRARIES})
