#include "SuperImpose/SISkeleton.h"

#include <iostream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

SISkeleton::SISkeleton() :
    hand_part_({ "palm", "thumb", "index", "medium" })
{
    std::cout << log_ID_ << "Initialization completed!" << std::endl;
}

SISkeleton::SISkeleton(const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy) :
    SISkeleton()
{
    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    setProjectionMatrix(cam_fx, cam_fy, cam_cx, cam_cy);
}


SISkeleton::~SISkeleton()
{ }


bool SISkeleton::superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    cam_pos_ = glm::make_vec3(cam_x);
    root_to_eye_ = glm::transpose(glm::mat3(glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])))));

    glm::vec2 ee_px = getWorldToPixel((objpos_map.find(hand_part_.front())->second).data());

    cv::Point endeffector_point(static_cast<int>(ee_px.x), static_cast<int>(ee_px.y));
    cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

    for (auto part = ++hand_part_.cbegin(); part != hand_part_.cend(); ++part)
    {
        cv::Point base_line = endeffector_point;
        for (auto map = objpos_map.equal_range(*part).first; map != objpos_map.equal_range(*part).second; ++map)
        {
            glm::vec2 joint_px = getWorldToPixel((map->second).data());

            cv::Point joint_point(static_cast<int>(joint_px.x), static_cast<int>(joint_px.y));

            cv::circle(img, joint_point, 3, cv::Scalar(0, 0, 255), 4);
            cv::line(img, base_line, joint_point, cv::Scalar(255, 0, 0), 2);

            base_line = joint_point;
        }
    }

    return true;
}


bool SISkeleton::setProjectionMatrix(const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy)
{

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921      0.0     162.202    0.0
                                   0.0      232.43    125.738    0.0
                                   0.0        0.0       1.0      0.0)
       Remember that GLM is column-major.                             */
    projection_ = glm::mat3(cam_fx,     0.0f,       0.0f,
                            0.0f,       cam_fy,     0.0f,
                            cam_cx,     cam_cy,     1.0f);

    return true;
}


glm::vec2 SISkeleton::getWorldToPixel(const double* world_point)
{
    glm::vec3 pos = glm::make_vec3(world_point);
    glm::vec3 px_h = projection_ * (root_to_eye_ * (pos - cam_pos_));

    return glm::vec2(px_h) / px_h.z;
}
