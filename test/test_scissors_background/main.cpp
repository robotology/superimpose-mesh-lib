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
    std::string log_ID = "[Test - Scissors - Background]";
    std::cout << log_ID << "This test checks whether the present machine supports GL_SCISSOR_TEST." << std::endl;
    std::cout << log_ID << "The same mesh will be rendered on 2 different viewports with a background texture." << std::endl;

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
                 2,
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
    objposes.push_back(objpose_map);

    double cam_x[] = { 0, 0, 0 };
    double cam_o[] = { 1.0, 0, 0, 0 };

    cv::Mat img = cv::imread("./space.png");
    si_cad.setBackgroundOpt(true);
    si_cad.superimpose(objposes, cam_x, cam_o, img);
    cv::imwrite("./test_scissors_background_Space_Invader.jpg", img);

    return EXIT_SUCCESS;
}
