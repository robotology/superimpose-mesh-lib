/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <exception>
#include <iostream>
#include <string>

#include <SuperimposeMesh/SICAD.h>

int main()
{
    std::string log_ID = "[Test - HDPI]";
    std::cout << log_ID << "This is a FAKE TEST to check whether the present machine sports HiDPI monitor or not." << std::endl;
    std::cout << log_ID << "A runtime error may be caught, but the test will always pass." << std::endl;
    std::cout << log_ID << "HiDPI should be handled automatically by the library." << std::endl  << std::endl;

    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34;
    const float        cam_cy     = 120;

    try
    {
        SICAD si_cad(SICAD::ModelPathContainer(),
                     cam_width, cam_height,
                     cam_fx, cam_fy, cam_cx, cam_cy);
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << log_ID << "Caught error:" << std::endl << e.what();
    }

    std::cout << log_ID << "If the rendered image size is greater than the given image size, then the library is not able to handle HiDPI on your machine!" << std::endl;
    std::cout << log_ID << "Should this occur, please open an issue on the GitHub library repository!" << std::endl;

    return EXIT_SUCCESS;
}
