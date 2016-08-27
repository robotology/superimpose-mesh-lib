#ifndef SUPERIMPOSESKELETONCADTHREAD_H
#define SUPERIMPOSESKELETONCADTHREAD_H

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iCub/iKin/iKinFwd.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::iKin;

class SuperimposeHandSkeletonThread : public Thread
{
private:
    const ConstString log_ID;
    const ConstString laterality;
    const ConstString camera;
    const int camsel;

    PolyDriver &arm_remote_driver;
    PolyDriver &arm_cartesian_driver;
    PolyDriver &gaze_driver;

    IEncoders *itf_arm_encoders;
    int num_arm_enc;
    ICartesianControl *itf_arm_cart;
    IGazeControl *itf_head_gaze;

    iCubFinger finger[3];

    BufferedPort<ImageOf<PixelRgb>> inport_skeleton_img;
    BufferedPort<ImageOf<PixelRgb>> outport_skeleton_img;
    BufferedPort<Bottle> port_cam_pose;

public:
    SuperimposeHandSkeletonThread(const ConstString &laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver);

    bool threadInit   ();
    void run          ();
    void onStop       ();
    void threadRelease();
};

#endif /* SUPERIMPOSESKELETONCADTHREAD_H */
