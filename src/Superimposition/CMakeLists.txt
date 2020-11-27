#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-fwdkin-ins-si)

find_package(BayesFilters REQUIRED)

find_package(OpenCV REQUIRED)

find_package(RobotsIO REQUIRED)

find_package(RobotsViz REQUIRED)

find_package(YARP REQUIRED COMPONENTS os sig)

set(${EXE_TARGET_NAME}_HDR)

set(${EXE_TARGET_NAME}_SRC
    src/Superimpose.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME}
                      BayesFilters::BayesFilters
                      RobotsIO::RobotsIO
                      RobotsViz::RobotsViz
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${OpenCV_LIBS}
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
