/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cassert>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <SuperimposeMesh/SICAD.h>


int main()
{
    std::string log_ID = "[Test - Public interfaces]";
    std::cout << log_ID << "This test checks whether SICAD public methods behave as expected." << std::endl;
    std::cout << log_ID << "A single mesh will be rendered on 1 viewport." << std::endl;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./Space_Invader.obj");

    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34;
    const float        cam_cy     = 120;

    SICAD si_cad(obj,
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

    std::vector<Superimpose::ModelPoseContainer> objposes;
    objposes.push_back(objpose_map);

    cv::Mat img;
    double cam_x[] = { 0, 0, 0 };
    double cam_o[] = { 1.0, 0, 0, 0 };


    unsigned int num_test = 1;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getOglWindowShouldClose()...";
    if (si_cad.getOglWindowShouldClose())
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.superimpose(objpose_map, cam_x, cam_o, img)...";
    if (!si_cad.superimpose(objpose_map, cam_x, cam_o, img))
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.superimpose(objposes, cam_x, cam_o, img)...";
    if (!si_cad.superimpose(objposes, cam_x, cam_o, img))
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getPBOs()..." << std::endl;
    const GLuint* pbo;
    size_t pbo_number;
    std::tie(pbo, pbo_number) = si_cad.getPBOs();
    for (size_t i = 0; i < pbo_number; ++i)
        std::cout << "PBO[" << i << "] = " << pbo[i] << "." << std::endl;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getPBO(index)..." << std::endl;
    bool got_pbo = false;
    GLuint pbo_value;
    for (size_t i = 0; i < pbo_number; ++i)
    {
        std::tie(got_pbo, pbo_value) = si_cad.getPBO(i);
        if (!got_pbo)
            return EXIT_FAILURE;
        std::cout << "PBO[" << i << "] = " << pbo_value << "." << std::endl;
    }
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy)...";
    if (!si_cad.setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy))
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.setBackgroundOpt(true)...";
    si_cad.setBackgroundOpt(true);
    std::cout << "...passed." << std::endl;
    ++num_test;

    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getBackgroundOpt()...";
    if (!si_cad.getBackgroundOpt())
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.setWireframeOpt(true)...";
    si_cad.setWireframeOpt(true);
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getWireframeOpt()...";
    if (!si_cad.getWireframeOpt())
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.setMipmapsOpt(SICAD::MIPMaps::linear)...";
    si_cad.setMipmapsOpt(SICAD::MIPMaps::linear);
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getMipmapsOpt()...";
    if (si_cad.getMipmapsOpt() != SICAD::MIPMaps::linear)
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getTilesNumber()...";
    if (si_cad.getTilesNumber() != 1)
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getTilesRows()...";
    if (si_cad.getTilesRows() != 1)
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    std::cout << log_ID << " " << num_test << " " << "testing si_cad.getTilesCols()...";
    if (si_cad.getTilesCols() != 1)
        return EXIT_FAILURE;
    std::cout << "...passed." << std::endl;
    ++num_test;


    return EXIT_SUCCESS;
}
