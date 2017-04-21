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
#include <SuperImpose/SICAD.h>


int main()
{
    std::string log_ID = "[TEST-SCISSORS]";
    std::cout << log_ID << "This test checks whether the present machine supports GL_SCISSOR_TEST." << std::endl;
    std::cout << log_ID << "The same mesh will be rendered on 2 differente viewport." << std::endl;

    SuperImpose::ObjFileMap obj;
    obj.emplace("palm", "./Space_Invader.obj");

    const unsigned int cam_width_  = 320;
    const unsigned int cam_height_ = 240;
    const float        cam_fx_     = 257.34;
    const float        cam_cx_     = 160;
    const float        cam_fy_     = 120;
    const float        cam_cy_     = 257.34;

    SICAD* si_cad;
    try
    {
        si_cad = new SICAD(obj, cam_width_, cam_height_, 2, ".",
                           cam_fx_, cam_fy_, cam_cx_, cam_cy_);
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << log_ID << "Caught error:" << std::endl << e.what();
        return EXIT_FAILURE;
    }

    std::vector<SuperImpose::ObjPoseMap> objposes;
    SuperImpose::ObjPoseMap              objpose_map;
    SuperImpose::ObjPose                 obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = 0;
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;
    objpose_map.emplace("palm", obj_pose);
    objposes.push_back(objpose_map);
    objposes.push_back(objpose_map);

    double cam_x[] = {0.1, 0, 0.15};
    double cam_o[] = {-0.57735, -0.57735, 0.57735, 2.0944};

    cv::Mat img;
    si_cad->superimpose(objposes, cam_x, cam_o, img);
    cv::imwrite("./Space_Invader.jpg", img);

    delete si_cad;

    return EXIT_SUCCESS;
}
