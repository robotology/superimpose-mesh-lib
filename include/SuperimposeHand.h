#ifndef SUPERIMPOSEHAND_H
#define SUPERIMPOSEHAND_H

#include <string>
#include <map>
#include <unordered_map>
#include <utility>

#include <opencv2/core/core.hpp>

class SuperimposeHand
{
public:

    typedef typename std::unordered_map<std::string, std::string> ObjFileMap;

    typedef typename std::multimap<std::string, std::pair<double *, double *>> ObjPoseMap;

    SuperimposeHand ();

    ~SuperimposeHand();

protected:
    virtual bool superimposeHand(ObjPoseMap obj2pos_map,
                                 const double * cam_x, const double * cam_o,
                                 cv::Mat img) = 0;

};

#endif /* SUPERIMPOSEHAND_H */
