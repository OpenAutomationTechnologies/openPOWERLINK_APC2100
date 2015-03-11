################################################################################
#
# CMake file of APC/PPC2100 firmware update tool
#
# Copyright (c) 2015, Kalycito Infotech Private Limited
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

################################################################################
# Setup project and generic options

PROJECT(fwupdate_tool C)
MESSAGE(STATUS "Configuring fwupdate_tool")

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

INCLUDE(../common/cmake/options.cmake)

# Set generic sources and include directories for this demo
SET(DEMO_SOURCES
    ${DEMO_SOURCE_DIR}/main.c
    ${DEMO_SOURCE_DIR}/ctrlu.c
    ${CONTRIB_SOURCE_DIR}/console/printlog.c
    ${CONTRIB_SOURCE_DIR}/getopt/getopt.c
    ${OPLK_ROOT_DIR}/stack/src/user/ctrl/ctrlucal-winioctl.c
    ${OPLK_ROOT_DIR}/stack/src/common/debugstr.c
    )

INCLUDE_DIRECTORIES(
    ${DEMO_SOURCE_DIR}
    ${OPLK_INCLUDE_DIR}
    ${OPLK_ROOT_DIR}/stack/proj/windows/liboplkmnapp-pcieintf
    )

ADD_DEFINITIONS(-DCONFIG_MN)

################################################################################
# Setup compile definitions depending on configuration

################################################################################
# Setup the architecture specific definitions

IF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    include (windows.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} is not supported!")
ENDIF()

################################################################################
# Group Source Files

SOURCE_GROUP("Demo Sources" FILES ${DEMO_SOURCES})
SOURCE_GROUP("Architecture Specific Sources" FILES ${DEMO_ARCH_SOURCES})

################################################################################
# Set the executable

ADD_EXECUTABLE(fwupdate_tool ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES} ${CMAKE_BINARY_DIR}/image.bin)
SET_PROPERTY(TARGET fwupdate_tool
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_BINARY_DIR}/image.bin
                   COMMAND ${CMAKE_COMMAND} -E copy ${DEMO_SOURCE_DIR}/image.bin ${CMAKE_BINARY_DIR}/image.bin
                   DEPENDS ${DEMO_SOURCE_DIR}/image.bin
                   VERBATIM
                   )

################################################################################
# Libraries to link
TARGET_LINK_LIBRARIES(fwupdate_tool ${ARCH_LIBRARIES})

################################################################################
# Installation rules

INSTALL(TARGETS fwupdate_tool RUNTIME DESTINATION ${CMAKE_PROJECT_NAME})
INSTALL(FILES ${CMAKE_BINARY_DIR}/image.bin DESTINATION ${CMAKE_PROJECT_NAME})
