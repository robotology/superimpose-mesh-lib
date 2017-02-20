#ifndef SUPERIMPOSESKELETON_H
#define SUPERIMPOSESKELETON_H

#include "SuperImpose.h"

#include <list>
#include <string>

#include <glm/glm.hpp>


class SISkeleton : public SuperImpose
{
public:
    SISkeleton(const float eye_fx, const float eye_fy, const float eye_cx, const float eye_cy);

    ~SISkeleton();

    bool superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img);

private:
    const std::string      log_ID_;

    std::list<std::string> hand_part_;
    glm::mat3              projection_;
    glm::mat3              root_to_eye_;
    glm::vec3              cam_pos_;

    glm::vec2              getWorldToPixel(const double* world_point);
};

#endif /* SUPERIMPOSESKELETON_H */
