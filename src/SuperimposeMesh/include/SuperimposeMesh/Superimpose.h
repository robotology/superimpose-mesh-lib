/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef SUPERIMPOSEHAND_H
#define SUPERIMPOSEHAND_H

#include <string>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>


class Superimpose
{
public:
    typedef typename std::vector<double>                   ModelPose;
    typedef typename std::multimap<std::string, ModelPose> ModelPoseContainer;
    typedef typename std::pair<std::string, ModelPose>     ModelPoseContainerElement;

    virtual ~Superimpose() { };

    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img) = 0;
};

#endif /* SUPERIMPOSEHAND_H */
