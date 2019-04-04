/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <SuperimposeMesh/SICAD.h>


int main()
{
    std::string log_ID = "[Test - Multiple windows - Moving Object]";
    std::cout << log_ID << "This test checks whether the present machine can render a moving object properly using OpenGL." << std::endl;
    std::cout << log_ID << "A single mesh will be rendered on 1 viewport." << std::endl;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./Space_Invader.obj");

    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34f;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34f;
    const float        cam_cy     = 120;

    SICAD si_cad_1(obj,
                   cam_width, cam_height,
                   cam_fx, cam_fy, cam_cx, cam_cy,
                   1,
                   ".");

    SICAD si_cad_2(obj,
                   cam_width, cam_height,
                   cam_fx, cam_fy, cam_cx, cam_cy,
                   1,
                   ".");

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -0.1;
    obj_pose[3] = 0;
    obj_pose[4] = 1.0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;

    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("alien", obj_pose);

    double cam_x[] = { 0, 0, 0 };
    double cam_o[] = { 1.0, 0, 0, 0 };

    for (double ang = 0; ang <= 2*M_PI; ang += 0.082)
    {
        cv::Mat img;

        obj_pose[6] = ang;
        objpose_map.clear();
        objpose_map.emplace("alien", obj_pose);

        si_cad_1.superimpose(objpose_map, cam_x, cam_o, img);
        si_cad_2.superimpose(objpose_map, cam_x, cam_o, img);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    return EXIT_SUCCESS;
}
