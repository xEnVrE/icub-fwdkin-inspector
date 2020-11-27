#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-fwdkin-ins-gt)

find_package(ArucoGroundTruthLibrary REQUIRED)

find_package(BayesFilters REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(YARP REQUIRED COMPONENTS os sig)

set(${EXE_TARGET_NAME}_HDR)

set(${EXE_TARGET_NAME}_SRC
    src/HandGroundTruth.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME}
                      ArucoGroundTruthLibrary::ArucoGroundTruthLibrary
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${OpenCV_LIBS}
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
