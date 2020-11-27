/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoGroundTruthLibrary/ArucoMeasurement.h>
#include <ArucoGroundTruthLibrary/ArucoBoardMeasurement.h>
#include <ArucoGroundTruthLibrary/ArucoMarkerMeasurement.h>
#include <ArucoGroundTruthLibrary/ThreePointReverseLinkMeasurement.h>

#include <BayesFilters/FilteringAlgorithm.h>

#include <Eigen/Dense>

#include <RobotsIO/Camera/iCubCamera.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

#include <opencv2/aruco.hpp>

#include <yarp/sig/Vector.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace ArucoGroundTruthLibrary;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace Eigen;


class ReverseArucoMeasurement : public bfl::FilteringAlgorithm
{
public:
    ReverseArucoMeasurement(const std::string& type, const std::string& laterality) :
        type_(type),
        laterality_(laterality)
    {}


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        if ((type_ != "marker") && (type_ != "board_0") && (type_ != "board_1"))
            throw(std::runtime_error("ReverseArucoMeasurement::initialization. Error: unknown type " + type_ + "."));

        const std::string port_prefix = "/" + type_ + "/" + laterality_;

        /* Camera. */
        camera_ = std::unique_ptr<iCubCamera>
        (
            new iCubCamera("icub", laterality_, port_prefix, "", "")
        );

        /* Probes .*/
        pose_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>("/" + port_prefix + "/pose:o")
        );

        pose_w_camera_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>("/" + port_prefix + "/pose_w_camera:o")
        );

        image_probe_ = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
        (
            new YarpImageOfProbe<yarp::sig::PixelRgb>("/" + port_prefix + "/image:o")
        );

        /* aruco measurement. */
        std::unique_ptr<ArucoMeasurement> aruco;
        if (type_ == "marker")
        {
            const double marker_length = 0.05;
            aruco = std::unique_ptr<ArucoMarkerMeasurement>
            (
                new ArucoMarkerMeasurement(cv::aruco::DICT_4X4_50, marker_length, std::move(camera_))
            );
        }
        else if ((type_ == "board_0") || (type_ == "board_1"))
        {
            std::size_t n_x = 2;
            std::size_t n_y = 2;
            std::size_t dictionary_offset = 0;
            const double intra_marker = 0.005;
            const double marker_length = 0.02;

            if (type_ == "board_1")
            {
                n_x = 1;
                n_y = 2;
                dictionary_offset = 4;
            }

            aruco = std::unique_ptr<ArucoBoardMeasurement>
            (
                new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, std::move(camera_))
            );
        }
        aruco->set_probe("image_output", std::move(image_probe_));

        /* Three point reverse link measurement. */
        Vector3d point_0(-0.0303521, 0.0243051, 0.0389612);
        Vector3d point_1(-0.0303521, -0.0256949, 0.0389612);
        Vector3d point_2(0.0196458, 0.0243051, 0.0384987);
        Vector3d corner_offset(0.0025, 0.0025, 0.0);
        if (type_ == "board_1")
        {
            point_0 = Vector3d(-0.0437343, 0.000461959, -0.00474314);
            point_1 = Vector3d(-0.0437343, 0.0254620, -0.00474314);
            point_2 = Vector3d(0.00626574, 0.000461959, -0.00474314);
        }

        link_measurement_ = std::unique_ptr<ThreePointReverseLinkMeasurement>
        (
            new ThreePointReverseLinkMeasurement(point_0, point_1, point_2, corner_offset, std::move(aruco))
        );
        link_measurement_->set_probe("pose", std::move(pose_probe_));
        link_measurement_->set_probe("pose_w_camera", std::move(pose_w_camera_probe_));

        return true;
    }


    void filteringStep() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        link_measurement_->freeze();
    }


    bool runCondition() override
    {
        return true;
    }

private:

    const std::string type_;

    const std::string laterality_;

    std::unique_ptr<ReverseLinkMeasurement> link_measurement_;

    /* Camera input. */

    std::unique_ptr<iCubCamera> camera_;

    /* Output. */

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>> pose_probe_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Affine>>> pose_w_camera_probe_;

    std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_;

    /* Log name. */

    const std::string port_prefix_ = "icub-fwdkin-ins-si";
};


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: icub-fwdkin-ins-gt <type> <laterality> "  << std::endl;
        std::cout << "          <type> can be 'marker', 'board_0' or 'board_1'"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string type = std::string(argv[1]);
    const std::string laterality = std::string(argv[2]);

    ReverseArucoMeasurement test(type, laterality);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
