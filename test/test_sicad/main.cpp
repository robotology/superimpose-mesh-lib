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

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <SuperimposeMesh/SICAD.h>


int main()
{
    std::string log_ID = "[Test - SICAD]";
    std::cout << log_ID << "This test checks whether the present machine can render properly using OpenGL." << std::endl;
    std::cout << log_ID << "A single mesh will be rendered on 1 viewport." << std::endl;

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

    SICAD si_cad(obj, cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, 1);


    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -0.10;
    obj_pose[3] = 0;
    obj_pose[4] = 1.0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;


    Superimpose::ModelPoseContainer alien_objpose_map;
    alien_objpose_map.emplace("alien", obj_pose);

    /* Space invader alien */
    cv::Mat img_rendered_alien;
    cv::Mat img_rendered_alien_depth;

    si_cad.superimpose(alien_objpose_map, cam_x, cam_o, img_rendered_alien, img_rendered_alien_depth);

    cv::imwrite("./test_sicad_alien.png", img_rendered_alien);
    cv::imwrite("./test_sicad_alien_depth.tiff", img_rendered_alien_depth);

    std::cout << "Pos (120, 120) " << img_rendered_alien_depth.at<float>(119, 119) << std::endl;
    std::cout << "Pos (120, 160) " << img_rendered_alien_depth.at<float>(119, 159) << std::endl;
    std::cout << "Pos (120, 300) " << img_rendered_alien_depth.at<float>(119, 199) << std::endl;
    std::cout << "Pos (1, 1) "	   << img_rendered_alien_depth.at<float>(0, 0)	   << std::endl;

    cv::Mat img_ground_truth_alien = cv::imread("./gt_sicad_alien.png");

    if (!utils::compareImages(img_rendered_alien, img_ground_truth_alien))
    {
        std::cerr << log_ID << "[Alien] Rendered and ground truth images are different." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << log_ID << "[Alien] Rendered and ground truth images are identical. Saving rendered image for visual inspection." << std::endl;
    /* ******************* */


    /* Space invader alien with space background */
    cv::Mat img_rendered_alien_space = cv::imread("./space.png");

    si_cad.setBackgroundOpt(true);
    si_cad.superimpose(alien_objpose_map, cam_x, cam_o, img_rendered_alien_space);

    cv::imwrite("./test_sicad_alien_space.png", img_rendered_alien_space);

    cv::Mat img_ground_truth_alien_space = cv::imread("./gt_sicad_alien_space.png");

    if (!utils::compareImages(img_rendered_alien_space, img_ground_truth_alien_space))
    {
        std::cerr << log_ID << "[Alien + Background] Rendered and ground truth images are different." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << log_ID << "[Alien + Background] Rendered and ground truth images are identical. Saving rendered image for visual inspection." << std::endl;
    /* ******************************** */


    Superimpose::ModelPoseContainer textured_alien_objpose_map;
    textured_alien_objpose_map.emplace("textured_alien", obj_pose);

    /* Space invader textured alien */
    cv::Mat img_rendered_textured_alien;

    si_cad.superimpose(textured_alien_objpose_map, cam_x, cam_o, img_rendered_textured_alien);

    cv::imwrite("./test_sicad_textured_alien.png", img_rendered_textured_alien);

    cv::Mat img_ground_truth_textured_alien = cv::imread("./gt_sicad_textured_alien.png");

    if (!utils::compareImages(img_rendered_textured_alien, img_ground_truth_textured_alien))
    {
        std::cerr << log_ID << "[Textured alien] Rendered and ground truth images are different." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << log_ID << "[Textured alien] Rendered and ground truth images are identical. Saving rendered image for visual inspection." << std::endl;
    /* **************************** */


    /* Space invader textured alien with space background */
    cv::Mat img_rendered_textured_alien_space = cv::imread("./space.png");

    si_cad.setBackgroundOpt(true);
    si_cad.superimpose(textured_alien_objpose_map, cam_x, cam_o, img_rendered_textured_alien_space);

    cv::imwrite("./test_sicad_textured_alien_space.png", img_rendered_textured_alien_space);

    cv::Mat img_ground_truth_textured_alien_space = cv::imread("./gt_sicad_textured_alien_space.png");

    if (!utils::compareImages(img_rendered_textured_alien_space, img_ground_truth_textured_alien_space))
    {
        std::cerr << log_ID << "[Textured alien + Background] Rendered and ground truth images are different." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << log_ID << "[Textured alien + Background] Rendered and ground truth images are identical. Saving rendered image for visual inspection." << std::endl;
    /* ************************************************** */


    return EXIT_SUCCESS;
}
