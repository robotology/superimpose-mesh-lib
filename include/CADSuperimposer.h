#ifndef CADSUPERIMPOSER_H
#define CADSUPERIMPOSER_H

#include <unordered_map>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>
#include <iCub/iKin/iKinFwd.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "SICAD.h"
//#include "ThreadControllerSHC.h"


class CADSuperimposer : public yarp::os::Thread
{
private:
    const yarp::os::ConstString     log_ID_;

    // TODO: rivedere il costruttore ed instanziare i PolyDriver in ThreadInit.
    const yarp::os::ConstString      laterality_;
    const yarp::os::ConstString      camera_;
    const int                        camsel_;
    yarp::dev::PolyDriver          & arm_remote_driver_;
    yarp::dev::PolyDriver          & arm_cartesian_driver_;
    yarp::dev::PolyDriver          & gaze_driver_;
    const SuperImpose::ObjFileMap  & cad_hand_;
    GLFWwindow                    *& window_;

    yarp::dev::IEncoders          *  itf_arm_encoders_;
    yarp::dev::ICartesianControl  *  itf_arm_cart_;
    yarp::dev::IGazeControl       *  itf_head_gaze_;
    int                              num_arm_enc_;
    float                            EYE_FX_;
    float                            EYE_FY_;
    float                            EYE_CX_;
    float                            EYE_CY_;

    iCub::iKin::iCubFinger           finger_[3];

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img_;
    yarp::os::BufferedPort<yarp::os::Bottle>                        port_cam_pose_;

    SICAD     drawer_;

//    ThreadControllerSHC helper;
    yarp::os::Port port_command_;

    bool setCommandPort();

public:
    CADSuperimposer(const yarp::os::ConstString & laterality, const yarp::os::ConstString & camera, yarp::dev::PolyDriver & arm_remote_driver, yarp::dev::PolyDriver & arm_cartesian_driver, yarp::dev::PolyDriver & gaze_driver, const SuperImpose::ObjFileMap & cad_hand, GLFWwindow *& window);

    bool threadInit   ();
    void run          ();
    void onStop       ();
    void threadRelease();
};

#endif /* CADSUPERIMPOSER */
