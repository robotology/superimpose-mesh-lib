#ifndef SUPERIMPOSESKELETON_H
#define SUPERIMPOSESKELETON_H

#include "Superimpose.h"

#include <list>
#include <string>

#include <glm/glm.hpp>


class SISkeleton : public Superimpose
{
public:
    SISkeleton();
    
    SISkeleton(const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy);

    ~SISkeleton();

    bool superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img);

    bool setProjectionMatrix(const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy);

protected:
    glm::vec2              getWorldToPixel(const double* world_point);

private:
    const std::string      log_ID_ = "[SI-Skeleton]";

    std::list<std::string> hand_part_;
    glm::mat3              projection_;
    glm::mat3              root_to_eye_;
    glm::vec3              cam_pos_;
};

#endif /* SUPERIMPOSESKELETON_H */
