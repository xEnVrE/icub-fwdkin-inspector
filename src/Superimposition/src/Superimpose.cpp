/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <BayesFilters/FilteringAlgorithm.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>

#include <RobotsViz/SIiCubHand.h>

#include <opencv2/opencv.hpp>

#include <yarp/sig/Image.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <iostream>

using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace RobotsViz;


class TestSuperimposition : public bfl::FilteringAlgorithm
{
public:
    TestSuperimposition(const std::string& robot_name, const std::string& hand_laterality, const bool use_analogs, const std::string& camera_laterality, const bool& use_camera_pose, const std::string& meshes_path, const bool& use_debug_cover) :
        robot_name_(robot_name),
        hand_laterality_(hand_laterality),
        use_analogs_(use_analogs),
        camera_laterality_(camera_laterality),
        use_camera_pose_(use_camera_pose),
        meshes_path_(meshes_path),
        use_debug_cover_(use_debug_cover)
    {}


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        const std::string port_prefix = port_prefix_ + "/" + camera_laterality_ + "_camera" + "/" + hand_laterality_ + "_hand";

        /* Camera. */
        camera_ = std::shared_ptr<iCubCamera>
        (
            new iCubCamera(robot_name_, camera_laterality_, port_prefix, "", "")
        );

        /* Probes .*/
        image_probe_ = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
        (
            new YarpImageOfProbe<yarp::sig::PixelRgb>("/" + port_prefix + "/image:o")
        );

        /* Rendering engine. */
        hand_ = std::unique_ptr<SIiCubHand>
        {
            new SIiCubHand(meshes_path_, robot_name_, hand_laterality_, port_prefix + "/si-icub-hand", use_analogs_, use_camera_pose_, camera_, use_debug_cover_)
        };


        return true;
    }


    void filteringStep() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        /* Get image from camera. */
        cv::Mat image_in;
        std::tie(std::ignore, image_in) = camera_->rgb(true);

        /* Render pose of hand. */
        bool valid_hand_rendering;
        cv::Mat render;
        std::tie(valid_hand_rendering, render) = hand_->render_image(true);
        if (!valid_hand_rendering)
        {
            std::cout << log_name_ << "::filteringStep. Error: cannot render hand on camera." << std::endl;

            return;
        }

        /* Transform to b/w mask. */
        cv::cvtColor(render, render, cv::COLOR_RGB2GRAY);
        cv::threshold(render, render, 128, 255, CV_THRESH_BINARY);

        /* Extract and draw contour. */
        std::vector<std::vector<cv::Point>> contours;
        findContours(render, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(image_in, contours, 0, cv::Scalar(0, 0, 255), 2);

        /* Set image within a probe for inspection. */
        image_probe_->set_data(image_in);
    }


    bool runCondition() override
    {
        return true;
    }

private:
    /* RGB input. */

    std::shared_ptr<Camera> camera_;

    /* Rendering. */

    std::unique_ptr<SIiCubHand> hand_;

    /* Output. */

    std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_;

    /* Robot. */

    const std::string robot_name_;

    const std::string hand_laterality_;

    const bool use_analogs_;

    /* Camera. */

    const std::string camera_laterality_;

    const bool use_camera_pose_;

    /* Hand meshes. */

    const std::string meshes_path_;

    const bool use_debug_cover_;

    /* Log name. */

    const std::string log_name_ = "iCubFwdkinInspector::Superimpose";

    const std::string port_prefix_ = "icub-fwdkin-ins-si";
};


int main(int argc, char** argv)
{
    if (argc != 8)
    {
        std::cout << "Synopsis: icub-fwdkin-ins-si <robot_name> <hand_laterality> <use_analogs> <camera_laterality> <use_camera_pose> <meshes_path> <use_debug_cover>"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string robot_name = std::string(argv[1]);
    const std::string hand_laterality = std::string(argv[2]);
    const bool use_analogs = (std::string(argv[3]) == "true");
    const std::string camera_laterality = std::string(argv[4]);
    const bool use_camera_pose = (std::string(argv[5]) == "true");
    const std::string meshes_path = std::string(argv[6]);
    const bool use_debug_cover = (std::string(argv[7]) == "true");

    TestSuperimposition test(robot_name, hand_laterality, use_analogs, camera_laterality, use_camera_pose, meshes_path, use_debug_cover);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
