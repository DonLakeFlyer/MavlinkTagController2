# Copyright 2012 Jared Boone
# Copyright 2013/2014 Benjamin Vernoux
# Copyright 2018 Andrea Montefusco IW0HDV <andrew@montefusco.com>

# This file is part of AirSpy (based on HackRF project).
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

# Based heavily upon the libftdi cmake setup.

set(INSTALL_DEFAULT_BINDIR "bin" CACHE STRING "Appended to CMAKE_INSTALL_PREFIX")

if(MSVC)
add_library(libgetopt_static STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../getopt/getopt.c
)
endif()

add_executable(airspyhf_zeromq_rx airspyhf_rx.c)
target_include_directories(airspyhf_zeromq_rx PRIVATE
    ${CMAKE_SOURCE_DIR}/shared
)
install(TARGETS airspyhf_zeromq_rx RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

set_target_properties(airspyhf_zeromq_rx PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(NOT libairspyhf_SOURCE_DIR)
include_directories(${LIBAIRSPYHF_INCLUDE_DIR})
LIST(APPEND TOOLS_LINK_LIBS ${LIBAIRSPYHF_LIBRARIES})
else()
LIST(APPEND TOOLS_LINK_LIBS airspyhf)
endif()

if(MSVC)
LIST(APPEND TOOLS_LINK_LIBS libgetopt_static)
endif()

LIST(APPEND TOOLS_LINK_LIBS -lm)

target_link_libraries(airspyhf_zeromq_rx ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_zeromq_rx PkgConfig::PC_ZMQ)
