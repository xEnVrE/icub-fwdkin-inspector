/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkiCubHand.h>

#include <cstdlib>

using namespace RobotsViz;


int main(int argc, char** argv)
{
    /* Get parameters from CLI. */
    if (argc != 5)
    {
        std::cout << "Synopsis: icub-fwdkin-ins-viewer <robot_name> <hand_laterality> <use_fingers> <use_analogs>"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string robot_name{argv[1]};
    const std::string hand_laterality{argv[2]};
    const bool use_fingers = (std::string{argv[3]} == "true");
    const bool use_analogs = (std::string{argv[4]} == "true");

    /* Initialize VTK container. */
    double fps = 30.0;
    VtkContainer container(1.0 / fps, 600, 600, true);

    /* Initialize VTK-based iCub hand rendering. */
    std::unique_ptr<VtkContent> vtk_hand = std::unique_ptr<VtkiCubHand>
    (
       new VtkiCubHand(robot_name, hand_laterality, "icub-fwdkin-ins-viewer/vtk_hand", use_fingers, use_analogs, {100.0 / 255.0, 100.0 / 255.0, 100.0 / 255.0}, 1.0)
    );
    container.add_content("vtk_hand", std::move(vtk_hand));

    /* Run the container. */
    container.run();

    return EXIT_SUCCESS;
}
