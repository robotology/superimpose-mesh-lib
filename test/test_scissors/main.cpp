/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <SuperimposeMesh/SICAD.h>


int main()
{
    std::string log_ID = "[Test - Scissors]";
    std::cout << log_ID << "This test checks whether the present machine supports GL_SCISSOR_TEST." << std::endl;
    std::cout << log_ID << "The same mesh will be rendered on 2 differente viewport." << std::endl;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./Space_Invader.obj");

    const unsigned int cam_width_  = 320;
    const unsigned int cam_height_ = 240;
    const float        cam_fx_     = 257.34;
    const float        cam_cx_     = 160;
    const float        cam_fy_     = 257.34;
    const float        cam_cy_     = 120;

    SICAD si_cad(obj,
                 cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                 2,
                 ".",
                 true);

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

    std::vector<Superimpose::ModelPoseContainer> objposes;
    objposes.push_back(objpose_map);
    objposes.push_back(objpose_map);

    double cam_x[] = {  0, 0, 0};
    double cam_o[] = {1.0, 0, 0, 0};

    cv::Mat img;
    si_cad.superimpose(objposes, cam_x, cam_o, img);
    cv::imwrite("./test_scissors_Space_Invader.jpg", img);

    return EXIT_SUCCESS;
}
