#include "SuperImpose/SISkeleton.h"

#include <iostream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240


SISkeleton::SISkeleton(const float EYE_FX, const float EYE_FY, const float EYE_CX, const float EYE_CY) :
    log_ID_("[SH-Skeleton]"), hand_part_({ "palm", "thumb", "index", "medium" })
{
    std::cout << log_ID_ << "Setting up OpenCV porjection matrices." << std::endl;

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921      0.0     162.202    0.0
                                   0.0      232.43    125.738    0.0
                                   0.0        0.0       1.0      0.0)
       Remember that GLM is column-major.                             */
    projection_ = glm::mat3(EYE_FX,     0.0f,       0.0f,
                            0.0f,       EYE_FY,     0.0f,
                            EYE_CX,     EYE_CY,     1.0f);

    std::cout << log_ID_ << "OpenCV projection matrices succesfully set up!" << std::endl;

    std::cout << log_ID_ << "Initialization completed!" << std::endl;
}


SISkeleton::~SISkeleton()
{
    std::cout << log_ID_ << "Deallocating OpenCV resources..." << std::endl;

    std::cout << log_ID_ << "OpenCV resource deallocation completed!" << std::endl;
}


bool SISkeleton::superimpose(const ObjPoseMap & obj2pos_map, const double * cam_x, const double * cam_o, cv::Mat & img)
// TODO: maybe create a ObjPosMap with translation only info and a ObjRotMap with rotation only info.
{
    cam_pos_ = glm::make_vec3(cam_x);
    root_to_eye_ = glm::transpose(glm::mat3(glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])))));

    glm::vec2 ee_px = getWorldToPixel((obj2pos_map.find(hand_part_.front())->second).data());

    cv::Point endeffector_point(static_cast<int>(ee_px.x), static_cast<int>(ee_px.y));
    cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

    for (auto part = ++hand_part_.cbegin(); part != hand_part_.cend(); ++part)
    {
        cv::Point base_line = endeffector_point;
        for (auto map = obj2pos_map.equal_range(*part).first; map != obj2pos_map.equal_range(*part).second; ++map)
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


glm::vec2 SISkeleton::getWorldToPixel(const double * world_point)
{
    glm::vec3 pos = glm::make_vec3(world_point);
    glm::vec3 px_h = projection_ * (root_to_eye_ * (pos - cam_pos_));
    return glm::vec2(px_h) / px_h.z;
}
