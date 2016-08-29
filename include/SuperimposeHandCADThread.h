#ifndef SUPERIMPOSEHANDCADTHREAD_H
#define SUPERIMPOSEHANDCADTHREAD_H

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
#include <glm/glm.hpp>

#include "ThreadControllerSHC.h"
#include "Model.h"
#include "Shader.h"

typedef std::unordered_map<std::string, yarp::os::ConstString>                           PartFileMap;
typedef std::unordered_map<std::string, std::pair<yarp::sig::Vector, yarp::sig::Vector>> HandPose;
typedef std::unordered_map<std::string, Model*>                                          HandModel;


class SuperimposeHandCADThread : public yarp::os::Thread
{
private:
    const yarp::os::ConstString log_ID;
    const yarp::os::ConstString laterality;
    const yarp::os::ConstString camera;
    yarp::dev::PolyDriver       &arm_remote_driver;
    yarp::dev::PolyDriver       &arm_cartesian_driver;
    yarp::dev::PolyDriver       &gaze_driver;
    const yarp::os::ConstString &shader_background_vert;
    const yarp::os::ConstString &shader_background_frag;
    const yarp::os::ConstString &shader_model_vert;
    const yarp::os::ConstString &shader_model_frag;
    const PartFileMap           &cad_hand;
    const int                   camsel;

    yarp::dev::IEncoders         *itf_arm_encoders;
    yarp::dev::ICartesianControl *itf_arm_cart;
    yarp::dev::IGazeControl      *itf_head_gaze;
    int                          num_arm_enc;
    float                        EYE_L_FX;
    float                        EYE_L_FY;
    float                        EYE_L_CX;
    float                        EYE_L_CY;

    iCub::iKin::iCubFinger finger[3];

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img;
    yarp::os::BufferedPort<yarp::os::Bottle>                        port_cam_pose;

    GLFWwindow *window;

    GLuint texture;
    GLuint vao;
    GLuint ebo;
    GLuint vbo;
    Shader *shader_background = nullptr;
    Shader *shader_cad = nullptr;

    HandModel hand_model;

    glm::mat4 root_to_ogl;
    glm::mat4 back_proj;
    glm::mat4 projection;

    ThreadControllerSHC helper;
    yarp::os::Port      port_command;

    
    bool setCommandPort();

public:
    SuperimposeHandCADThread(const yarp::os::ConstString &laterality, const yarp::os::ConstString &camera,
                             yarp::dev::PolyDriver &arm_remote_driver, yarp::dev::PolyDriver &arm_cartesian_driver, yarp::dev::PolyDriver &gaze_driver,
                             const yarp::os::ConstString &shader_background_vert, const yarp::os::ConstString &shader_background_frag,
                             const yarp::os::ConstString &shader_model_vert, const yarp::os::ConstString &shader_model_frag,
                             const std::unordered_map<std::string, yarp::os::ConstString> &cad_hand, GLFWwindow *window);

    bool threadInit   ();
    void run          ();
    void onStop       ();
    void threadRelease();
};

#endif /* SUPERIMPOSEHANDCADTHREAD_H */
