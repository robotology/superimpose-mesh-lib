#ifndef SUPERIMPOSEHANDCADTHREAD_H
#define SUPERIMPOSEHANDCADTHREAD_H

#include <unordered_map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iCub/iKin/iKinFwd.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "ThreadControllerSHC.h"
#include "Model.h"
#include "Shader.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::iKin;


class SuperimposeHandCADThread : public Thread
{
private:
    const ConstString log_ID;
    const ConstString laterality;
    const ConstString camera;
    PolyDriver &arm_remote_driver;
    PolyDriver &arm_cartesian_driver;
    PolyDriver &gaze_driver;
    const ConstString &shader_background_vert;
    const ConstString &shader_background_frag;
    const ConstString &shader_model_vert;
    const ConstString &shader_model_frag;
    const std::unordered_map<std::string, ConstString> &cad_hand;
    const int camsel;

    IEncoders *itf_arm_encoders;
    int num_arm_enc;
    ICartesianControl *itf_arm_cart;
    IGazeControl *itf_head_gaze;
    float EYE_L_FX;
    float EYE_L_FY;
    float EYE_L_CX;
    float EYE_L_CY;

    iCubFinger finger[3];
    typedef std::unordered_map<std::string, std::pair<Vector, Vector>> HandPose;

    BufferedPort<ImageOf<PixelRgb>> inport_renderer_img;
    BufferedPort<ImageOf<PixelRgb>> outport_renderer_img;
    BufferedPort<Bottle> port_cam_pose;

    GLFWwindow *window;

    GLuint texture;
    GLuint vao;
    GLuint ebo;
    GLuint vbo;
    Shader *shader_background = nullptr;
    Shader *shader_cad = nullptr;

    typedef std::unordered_map<std::string, Model*> HandModel;
    HandModel hand_model;

    glm::mat4 root_to_ogl;
    glm::mat4 back_proj;
    glm::mat4 projection;

    ThreadControllerSHC helper;
    Port port_command;

    bool setCommandPort();

public:
    SuperimposeHandCADThread(const ConstString &laterality, const ConstString &camera,
                             PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver,
                             const ConstString &shader_background_vert, const ConstString &shader_background_frag,
                             const ConstString &shader_model_vert, const ConstString &shader_model_frag,
                             const std::unordered_map<std::string, ConstString> &cad_hand, GLFWwindow *window);

    bool threadInit   ();
    void run          ();
    void onStop       ();
    void threadRelease();
};

#endif /* SUPERIMPOSEHANDCADTHREAD_H */
