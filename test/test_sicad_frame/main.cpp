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
    std::string log_ID = "[Test - SICAD - Frame]";
    std::cout << log_ID << "This test checks whether the present machine can render a frame using OpenGL." << std::endl;

    SICAD::ModelPathContainer objfile_map;

    const unsigned int cam_width_  = 320;
    const unsigned int cam_height_ = 240;
    const float        cam_fx_     = 257.34;
    const float        cam_cx_     = 160;
    const float        cam_fy_     = 257.34;
    const float        cam_cy_     = 120;

    SICAD si_cad(objfile_map,
                 cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                 1,
                 ".",
                 true);

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = 0;
    obj_pose[3] = 1.0;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;

    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("frame", obj_pose);

    std::vector<Superimpose::ModelPoseContainer> objposes;
    objposes.push_back(objpose_map);

    double cam_x[] = { 0, 0,  0 };
    double cam_o[] = { 1.0, 0,  0, 0 };

    cv::Mat img_1;
    si_cad.superimpose(objpose_map, cam_x, cam_o, img_1);
    cv::imwrite("./test_sicad_frame.jpg", img_1);

    return EXIT_SUCCESS;
}
