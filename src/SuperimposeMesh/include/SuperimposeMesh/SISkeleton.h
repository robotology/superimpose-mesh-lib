/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef SUPERIMPOSESKELETON_H
#define SUPERIMPOSESKELETON_H

#include "Superimpose.h"

#include <list>
#include <string>

#include <glm/glm.hpp>


class SISkeleton : public Superimpose
{
public:
    SISkeleton(const std::list<std::string>& skeleton_part, const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy);

    ~SISkeleton();

    bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img) override;

    bool setProjectionMatrix(const float cam_fx, const float cam_fy, const float cam_cx, const float cam_cy);

protected:
    glm::vec2 getWorldToPixel(const double* world_point);

private:
    const std::string log_ID_ = "[SI-Skeleton]";

    std::list<std::string> skeleton_part_;

    glm::mat3 projection_;

    glm::mat3 root_to_eye_;
    
    glm::vec3 cam_pos_;
};

#endif /* SUPERIMPOSESKELETON_H */
