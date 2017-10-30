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
    std::string log_ID = "[Test - Initialization and controls]";
    std::cout << log_ID << "This test checks whether SICAD class can be lazy initialized and that proper controls avoid the use of OpenGL prior to its initialization." << std::endl;
    std::cout << log_ID << "A single mesh will be rendered on 1 viewport." << std::endl;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./Space_Invader.obj");

    const unsigned int cam_width_  = 320;
    const unsigned int cam_height_ = 240;
    const float        cam_fx_     = 257.34;
    const float        cam_cx_     = 160;
    const float        cam_fy_     = 120;
    const float        cam_cy_     = 257.34;

    SICAD si_cad;

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = 0;
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;

    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("alien", obj_pose);

    std::vector<Superimpose::ModelPoseContainer> objposes;
    objposes.push_back(objpose_map);

    cv::Mat img;
    double cam_x[] = {0.0, 0.1, 0.1};
    double cam_o[] = {1.0,   0,   0, 0};


    std::cout << log_ID << "assert(!si_cad.getOglWindowShouldClose());...";
    assert(!si_cad.getOglWindowShouldClose());
    std::cout << "...passed." << std::endl;

    std::cout << log_ID << "assert(!si_cad.superimpose(objpose_map, cam_x, cam_o, img));...";
    assert(!si_cad.superimpose(objpose_map, cam_x, cam_o, img));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(!si_cad.superimpose(objpose_map, cam_x, cam_o, img, cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));...";
    assert(!si_cad.superimpose(objpose_map, cam_x, cam_o, img,
                               cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(!si_cad.superimpose(objposes, cam_x, cam_o, img));...";
    assert(!si_cad.superimpose(objposes, cam_x, cam_o, img));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(!si_cad.superimpose(objposes, cam_x, cam_o, img, cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));...";
    assert(!si_cad.superimpose(objposes, cam_x, cam_o, img,
                               cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(!si_cad.setProjectionMatrix(cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));...";
    assert(!si_cad.setProjectionMatrix(cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(si_cad.initSICAD(obj, cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_, 1, {1.0, 0.0, 0.0, 0.0}, '.', true));...";
    assert(si_cad.initSICAD(obj,
                            cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                            1,
                            {1.0, 0.0, 0.0, 0.0},
                            ".",
                            true));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(!si_cad.getOglWindowShouldClose());...";
    assert(!si_cad.getOglWindowShouldClose());
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(si_cad.superimpose(objpose_map, cam_x, cam_o, img));...";
    assert(si_cad.superimpose(objpose_map, cam_x, cam_o, img));
    std::cout << "...passed." << std::endl;


    std::cout << log_ID << "assert(si_cad.superimpose(objposes, cam_x, cam_o, img));...";
    assert(si_cad.superimpose(objposes, cam_x, cam_o, img));
    std::cout << "...passed." << std::endl;

    return EXIT_SUCCESS;
}
