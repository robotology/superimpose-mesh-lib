#ifndef SUPERIMPOSEHAND_H
#define SUPERIMPOSEHAND_H

#include <unordered_map>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "SiCHSkeleton.h"
#include "SiCHCAD.h"
#include "src/SuperimposeHandIDL.h"

typedef std::unordered_map<std::string, yarp::os::ConstString> PartFileMap;


class SuperimposeiCubHand : public yarp::os::RFModule,
                            public SuperimposeHandIDL
{
private:
    const yarp::os::ConstString log_ID;

    GLFWwindow * window;

    yarp::os::ConstString robot;

    bool start;
    bool init_position;
    bool freerunning;
    bool superimpose_skeleton;
    bool superimpose_mesh;

    yarp::dev::PolyDriver        rightarm_remote_driver;
    yarp::dev::IEncoders         *itf_rightarm_enc;
    yarp::dev::IPositionControl2 *itf_rightarm_pos;
    int                          num_rightarm_joint;
    int                          num_rightarm_enc;

    yarp::dev::PolyDriver        rightarm_cartesian_driver;
    yarp::dev::ICartesianControl *itf_rightarm_cart;

    yarp::dev::PolyDriver        head_remote_driver;
    yarp::dev::IPositionControl2 *itf_head_pos;
    int                          num_head_joint;

    yarp::dev::PolyDriver   gaze_driver;
    yarp::dev::IGazeControl *itf_head_gaze;

    SuperimposeHandSkeletonThread *trd_left_cam_skeleton = nullptr;

    SuperimposeHandCADThread *trd_left_cam_cad = nullptr;
    yarp::os::ConstString    shader_background_vert;
    yarp::os::ConstString    shader_background_frag;
    yarp::os::ConstString    shader_model_vert;
    yarp::os::ConstString    shader_model_frag;
    PartFileMap              cad_hand;

    yarp::os::Port port_command;

    yarp::sig::Matrix frontal_view_R;
    yarp::sig::Vector frontal_view_x;

    yarp::sig::Matrix table_view_R;
    yarp::sig::Vector table_view_x;

    double open_hand_joints[6];
    double closed_hand_joints[6];

    double radius;
    int    angle_ratio;
    double motion_time;
    double path_time;

    
    bool fileFound                     (const yarp::os::ConstString &file);
    bool setRightArmRemoteControlboard ();
    bool setRightArmCartesianController();
    bool setHeadRemoteControlboard     ();
    bool setGazeController             ();
    bool setTorsoDOF                   ();
    bool setCommandPort                ();
    bool moveFingers                   (const double (&joint)[6]);
    bool moveHand                      (const yarp::sig::Matrix &R, const yarp::sig::Vector &init_x);

protected:
    bool move_hand        ();
    bool move_hand_freerun();
    bool stop_hand        ();
    bool initial_position ();
    bool view_hand        ();
    bool open_fingers     ();
    bool close_fingers    ();
    bool view_skeleton    (const bool status);
    bool view_mesh        (const bool status);
    std::string quit      ();

public:
    SuperimposeHand     ();
    double getPeriod    () { return 0.0; }
    bool configure      (yarp::os::ResourceFinder &rf);
    void setWindow      (GLFWwindow *window) { this->window = window; }
    bool updateModule   ();
    bool close          ();    
};

#endif /* SUPERIMPOSEHAND_H */
