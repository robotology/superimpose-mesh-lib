#include "SHSkeleton.h"

#include <iostream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240


SHSkeleton::SHSkeleton(const float EYE_L_FX, const float EYE_L_FY, const float EYE_L_CX, const float EYE_L_CY) : _log_ID("[SH-Skeleton]"), _EYE_L_FX(EYE_L_FX), _EYE_L_FY(EYE_L_FY), _EYE_L_CX(EYE_L_CX), _EYE_L_CY(EYE_L_CY)
{
    _hand_part = { "palm",
                   "thumb",
                   "index",
                   "medium" };

    std::cout << _log_ID << "Setting up OpenCV porjection matrices." << std::endl;

    /* Predefined rotation matrices. */
//    _root_to_ocv = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
//                             1.0f, 0.0f, 0.0f, 0.0f,
//                             0.0f, 1.0f, 0.0f, 0.0f,
//                             0.0f, 0.0f, 0.0f, 1.0f);
    _root_to_ocv = glm::mat4(1.0f);

    _view = glm::mat4(1.0f);

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921 0.0     162.202 0.0
                                 0.0     232.43  125.738 0.0
                                 0.0     0.0     1.0     0.0) */
    _projection = glm::mat3x4(_EYE_L_FX,  0,          _EYE_L_CX,  0,
                              0,          _EYE_L_FY,  _EYE_L_CY,  0,
                              0,          0,                  1,  0);

    std::cout << _log_ID << "OpenCV projection matrices succesfully set up!" << std::endl;

    std::cout << _log_ID << "Initialization completed!" << std::endl;
}


SHSkeleton::~SHSkeleton()
{
    std::cout << _log_ID << "Deallocating OpenCV resources..." << std::endl;

    std::cout << _log_ID << "OpenCV resource deallocation completed!" << std::endl;
}


bool SHSkeleton::superimposeHand(ObjPoseMap obj2pos_map,
                                 const double * cam_x, const double * cam_o,
                                 cv::Mat img)
{
    glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 root_eye_o = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    _view = glm::lookAt(glm::mat3(_root_to_ocv) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                        glm::mat3(_root_to_ocv) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(root_eye_o) * glm::vec3(0.0f, 0.0f, 1.0f)),
                        glm::mat3(_root_to_ocv) * glm::mat3(root_eye_o) * glm::vec3(0.0f, -1.0f, 0.0f));

    /* 3D to Pixel */
//    glm::vec3 ee_px_h = _projection * _view * glm::vec4(glm::make_vec3((obj2pos_map.find("palm")->second).first), 1.0f);
//    glm::vec2 ee_px   = glm::vec2(ee_px_h) / ee_px_h.z;
    glm::vec2 ee_px = getWorldToPixel((obj2pos_map.find("palm")->second).first);

    cv::Point endeffector_point(static_cast<int>(ee_px.x), static_cast<int>(ee_px.y));
    cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

    cv::Point base_line = endeffector_point;
    for (auto part = _hand_part.cbegin(); part != _hand_part.cend(); ++part)
    {
        for (auto map = obj2pos_map.equal_range(*part).first; map != obj2pos_map.equal_range(*part).second; ++map)
        {
            glm::vec2 joint_px = getWorldToPixel((map->second).first);

            cv::Point joint_point(static_cast<int>(joint_px.x), static_cast<int>(joint_px.y));

            cv::circle(img, joint_point, 3, cv::Scalar(0, 0, 255), 4);

            cv::line(img, base_line, joint_point, cv::Scalar(255, 0, 0), 2);

            base_line = joint_point;
        }
    }

    return true;
}


glm::vec2 SHSkeleton::getWorldToPixel(double * world_point)
{
    glm::vec3 px_h = _projection * _view * glm::vec4(glm::make_vec3(world_point), 1.0f);
    return glm::vec2(px_h) / px_h.z;
}