#ifndef SUPERIMPOSEHAND_H
#define SUPERIMPOSEHAND_H

#include <string>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>

class SuperImpose
{
public:

    typedef typename std::unordered_map<std::string, std::string> ObjFileMap;

    typedef typename std::vector<double> ObjPose;

    typedef typename std::multimap<std::string, ObjPose> ObjPoseMap;

    ~SuperImpose() {};

    virtual bool Superimpose(const ObjPoseMap & obj2pos_map, const double * cam_x, const double * cam_o, cv::Mat & img) = 0;

};

#endif /* SUPERIMPOSEHAND_H */
