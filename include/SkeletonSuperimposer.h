#ifndef SKELETONSUPERIMPOSER_H
#define SKELETONSUPERIMPOSER_H

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Image.h>
#include <iCub/iKin/iKinFwd.h>


class SkeletonSuperimposer : public yarp::os::Thread
{
private:
    const yarp::os::ConstString log_ID;
    const yarp::os::ConstString laterality;
    const yarp::os::ConstString camera;
    const int                   camsel;

    yarp::dev::PolyDriver &arm_remote_driver;
    yarp::dev::PolyDriver &arm_cartesian_driver;
    yarp::dev::PolyDriver &gaze_driver;

    yarp::dev::IEncoders         *itf_arm_encoders;
    yarp::dev::ICartesianControl *itf_arm_cart;
    yarp::dev::IGazeControl      *itf_head_gaze;
    int                          num_arm_enc;

    iCub::iKin::iCubFinger finger[3];

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_skeleton_img;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_skeleton_img;
    yarp::os::BufferedPort<yarp::os::Bottle>                        port_cam_pose;

public:
    SkeletonSuperimposer(const yarp::os::ConstString &laterality, const yarp::os::ConstString &camera, yarp::dev::PolyDriver &arm_remote_driver, yarp::dev::PolyDriver &arm_cartesian_driver, yarp::dev::PolyDriver &gaze_driver);

    bool threadInit   ();
    void run          ();
    void onStop       ();
    void threadRelease();
};

#endif /* SKELETONSUPERIMPOSER_H */
