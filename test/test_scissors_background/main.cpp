/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <utils.h>

#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <SuperimposeMesh/SICAD.h>


int main()
{
    std::string log_ID = "[Test - Scissors - Background]";
    std::cout << log_ID << " This test checks whether the present machine supports GL_SCISSOR_TEST." << std::endl;
    std::cout << log_ID << " The same mesh will be rendered on 2 different viewports with a background texture." << std::endl;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./spaceinvader.obj");
    obj.emplace("textured_alien", "./spaceinvader_textured.obj");

    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34;
    const float        cam_cy     = 120;

    double cam_x[] = { 0, 0, 0 };
    double cam_o[] = { 1.0, 0, 0, 0 };

    SICAD si_cad(obj, cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, 2);


    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -0.1;
    obj_pose[3] = 0;
    obj_pose[4] = 1.0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;


    Superimpose::ModelPoseContainer alien_objpose_map;
    alien_objpose_map.emplace("alien", obj_pose);

    Superimpose::ModelPoseContainer textured_alien_objpose_map;
    textured_alien_objpose_map.emplace("textured_alien", obj_pose);

    std::vector<Superimpose::ModelPoseContainer> objposes;
    objposes.push_back(alien_objpose_map);
    objposes.push_back(textured_alien_objpose_map);


    cv::Mat img_rendered = cv::imread("./space.png");

    si_cad.setBackgroundOpt(true);
    si_cad.superimpose(objposes, cam_x, cam_o, img_rendered);

    cv::imwrite("./test_scissors_background.png", img_rendered);

    cv::Mat img_ground_truth = cv::imread("./gt_scissors_background.png");

    if (!utils::compareImages(img_rendered, img_ground_truth))
    {
        std::cerr << log_ID << " Rendered and ground truth images are different." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << log_ID << " Rendered and ground truth images are identical. Saving rendered image for visual inspection." << std::endl;

    return EXIT_SUCCESS;
}
