#ifndef SUPERIMPOSESKELETON_H
#define SUPERIMPOSESKELETON_H

#include "SuperImpose.h"

#include <list>
#include <string>

#include <glm/glm.hpp>


class SISkeleton : public SuperImpose
{
public:
    SISkeleton(const float EYE_FX, const float EYE_FY, const float EYE_CX, const float EYE_CY);

    ~SISkeleton();

    bool superimpose(const ObjPoseMap & obj2pos_map, const double * cam_x, const double * cam_o, cv::Mat & img);

private:
    const std::string      log_ID_;

    std::list<std::string> hand_part_;
    glm::mat3              projection_;
    glm::mat3              root_to_eye_;
    glm::vec3              cam_pos_;

    glm::vec2              getWorldToPixel(const double * world_point);
};

#endif /* SUPERIMPOSESKELETON_H */
