#ifndef SUPERIMPOSESKELETONCADTHREAD_H
#define SUPERIMPOSESKELETONCADTHREAD_H

#include "SuperImpose.h"

#include <list>
#include <string>

#include <glm/glm.hpp>

class SISkeleton : public SuperImpose
{
public:
    SISkeleton(const float EYE_L_FX, const float EYE_L_FY, const float EYE_L_CX, const float EYE_L_CY);

    ~SISkeleton();

protected:
    bool superimpose(ObjPoseMap obj2pos_map,
                     const double * cam_x, const double * cam_o,
                     cv::Mat img);

private:
    const std::string _log_ID;
    const float       _EYE_L_FX;
    const float       _EYE_L_FY;
    const float       _EYE_L_CX;
    const float       _EYE_L_CY;

    std::list<std::string> _hand_part;
    glm::mat4              _root_to_ocv;
    glm::mat4              _view;
    glm::mat4              _projection;

    glm::vec2 getWorldToPixel(double * world_point);
};

#endif /* SUPERIMPOSESKELETONCADTHREAD_H */
