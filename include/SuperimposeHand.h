#ifndef SUPERIMPOSEHAND_H
#define SUPERIMPOSEHAND_H

#include <unordered_map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "SuperimposeHandSkeletonThread.h"
#include "SuperimposeHandCADThread.h"
#include "src/SuperimposeHandIDL.h"

using namespace yarp::os;
using namespace yarp::dev;

class SuperimposeHand : public RFModule,
                        public SuperimposeHandIDL
{
private:
    const ConstString log_ID;

    GLFWwindow * window;

    ConstString robot;

    bool start;
    bool init_position;
    bool freerunning;
    bool superimpose_skeleton;
    bool superimpose_mesh;

    PolyDriver rightarm_remote_driver;
    IEncoders *itf_rightarm_enc;
    IPositionControl2 *itf_rightarm_pos;
    int num_rightarm_joint;
    int num_rightarm_enc;

    PolyDriver rightarm_cartesian_driver;
    ICartesianControl *itf_rightarm_cart;

    PolyDriver head_remote_driver;
    IPositionControl2 *itf_head_pos;
    int num_head_joint;

    PolyDriver gaze_driver;
    IGazeControl *itf_head_gaze;

    SuperimposeHandSkeletonThread *trd_left_cam_skeleton = nullptr;

    SuperimposeHandCADThread *trd_left_cam_cad = nullptr;
    ConstString shader_background_vert;
    ConstString shader_background_frag;
    ConstString shader_model_vert;
    ConstString shader_model_frag;
    std::unordered_map<std::string, ConstString> cad_hand;

    Port port_command;

    Matrix frontal_view_R;
    Vector frontal_view_x;

    Matrix table_view_R;
    Vector table_view_x;

    double open_hand_joints[6];
    double closed_hand_joints[6];

    double radius;
    int angle_ratio;
    double motion_time;
    double path_time;

    
    bool fileFound                     (const ConstString &file);
    bool setRightArmRemoteControlboard ();
    bool setRightArmCartesianController();
    bool setHeadRemoteControlboard     ();
    bool setGazeController             ();
    bool setTorsoDOF                   ();
    bool setCommandPort                ();
    bool moveFingers                   (const double (&joint)[6]);
    bool moveHand                      (const Matrix &R, const Vector &init_x);

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
    bool configure      (ResourceFinder &rf);
    void setWindow      (GLFWwindow *window) { this->window = window; }
    bool updateModule   ();
    bool interruptModule();
    bool close          ();
    
};

#endif /* SUPERIMPOSEHAND_H */
