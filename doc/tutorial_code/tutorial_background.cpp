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
    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34;
    const float        cam_cy     = 120;

    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./spaceinvader.obj");

    SICAD si_cad(obj, cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -0.1;
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;

    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("alien", obj_pose);

    double cam_x[] = {  0, 0, 0};
    double cam_o[] = {1.0, 0, 0, 0};

    /**
     * It's background time!
     * The steps are the same, but this time the cv::Mat is created from an
     * existing image and we instruct the SICAD class to use it as a background
     * with setBackgroundOpt(true).
     * That's it!
     **/
    cv::Mat img = cv::imread("./space.png");
    si_cad.setBackgroundOpt(true);
    si_cad.superimpose(objpose_map, cam_x, cam_o, img);
    cv::imwrite("./spaceinvader.jpg", img);

    return EXIT_SUCCESS;
}
