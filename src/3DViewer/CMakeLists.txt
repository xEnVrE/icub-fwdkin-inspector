#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-fwdkin-ins-viewer)

find_package(RobotsViz REQUIRED)

set(${EXE_TARGET_NAME}_HDR)

set(${EXE_TARGET_NAME}_SRC src/3DViewer.cpp)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME} RobotsViz::RobotsViz)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
