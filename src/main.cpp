#include <cmath>
#include <iostream>
#include <list>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Model.h"
#include "Shader.h"
#include "src/SuperimposeHandIDL.h"
#include "src/SuperimposeHandCADIDL.h"

#define PROJECT_NAME ConstString("superimpose_hand")
#define WINDOW_WIDTH 320
#define WINDOW_HEIGHT 240
#ifdef GLFW_RETINA
#define FRAMEBUFFER_WIDTH WINDOW_WIDTH*2
#define FRAMEBUFFER_HEIGHT WINDOW_HEIGHT*2
#else
#define FRAMEBUFFER_WIDTH WINDOW_WIDTH
#define FRAMEBUFFER_HEIGHT WINDOW_HEIGHT
#endif
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define NEAR 0.001f
#define FAR 1000.0f

GLFWwindow *window;

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;


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
    SuperimposeHandSkeletonThread(const ConstString &laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver) : log_ID("[SuperimposeHandSkeletonThread]"), laterality(laterality), camera(camera), camsel((camera == "left")? 0:1), arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver) {}

    bool threadInit() {
        yInfo() << log_ID << "Initializing hand skeleton drawing thread.";

        yInfo() << log_ID << "Setting interfaces";
        
        IControlLimits *itf_fingers_lim;
        arm_remote_driver.view(itf_fingers_lim);
        if (!itf_fingers_lim) {
            yError() << log_ID << "Error getting IControlLimits interface in thread.";
            return false;
        }

        arm_remote_driver.view(itf_arm_encoders);
        if (!itf_arm_encoders) {
            yError() << log_ID << "Error getting IEncoders interface.";
            return false;
        }
        itf_arm_encoders->getAxes(&num_arm_enc);

        arm_cartesian_driver.view(itf_arm_cart);
        if (!itf_arm_cart) {
            yError() << log_ID << "Error getting ICartesianControl interface in thread.";
            return false;
        }

        gaze_driver.view(itf_head_gaze);
        if (!itf_head_gaze) {
            yError() << log_ID << "Error getting IGazeControl interface.";
            return false;
        }
        
        yInfo() << log_ID << "Interfaces set!";

        yInfo() << log_ID << "Setting joint bounds for the fingers.";
        
        finger[0] = iCubFinger(laterality+"_thumb");
        finger[1] = iCubFinger(laterality+"_index");
        finger[2] = iCubFinger(laterality+"_middle");

        std::deque<IControlLimits*> temp_lim;
        temp_lim.push_front(itf_fingers_lim);
        for (int i = 0; i < 3; ++i) {
            if (!finger[i].alignJointsBounds(temp_lim)) {
                yError() << log_ID << "Cannot set joint bound for finger " + std::to_string(i) + ".";
                return false;
            }
        }
        
        yInfo() << log_ID << "Joint bound for finger set!";

        yInfo() << log_ID << "Opening ports for skeleton images.";
        
        if (!inport_skeleton_img.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+":i")) {
            yError() << log_ID << "Cannot open input image port for "+camera+".";
            return false;
        }

        if (!outport_skeleton_img.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+":o")) {
            yError() << log_ID << "Cannot open output image port for "+camera+".";
            return false;
        }
        
        yInfo() << log_ID << "Skeleton image ports succesfully opened!";

        yInfo() << log_ID << "Opening ports for "+camera+" camera pose.";
        
        if (!port_cam_pose.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+"/pose:o")) {
            yError() << log_ID << "Cannot open "+camera+" camera pose output port.";
            return false;
        }
        
        yInfo() << log_ID << "Port for "+camera+" camera succesfully opened!";

        yInfo() << log_ID << "Initialization completed!";

        return true;
    }

    void run() {
        Vector ee_x(3);
        Vector ee_o(4);
        Vector cam_x(3);
        Vector cam_o(4);

        while (!isStopping()) {

            ImageOf<PixelRgb> *imgin = inport_skeleton_img.read(true);
            itf_arm_cart->getPose(ee_x, ee_o);
            itf_head_gaze->getLeftEyePose(cam_x, cam_o);

            if (imgin != NULL) {
                ImageOf<PixelRgb> &imgout = outport_skeleton_img.prepare();
                imgout = *imgin;

                cv::Mat img = cv::cvarrToMat(imgout.getIplImage());

                Matrix Ha = axis2dcm(ee_o);
                ee_x.push_back(1.0);
                Ha.setCol(3, ee_x);

                Vector endeffector_pixel;
                itf_head_gaze->get2DPixel(camsel, ee_x, endeffector_pixel);

                cv::Point endeffector_point(static_cast<int>(endeffector_pixel[0]), static_cast<int>(endeffector_pixel[1]));
                cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

                Vector encs(static_cast<size_t>(num_arm_enc));
                Vector chainjoints;
                itf_arm_encoders->getEncoders(encs.data());
                for (unsigned int i = 0; i < 3; ++i) {
                    finger[i].getChainJoints(encs, chainjoints);
                    finger[i].setAng(CTRL_DEG2RAD * chainjoints);
                }

                for (unsigned int fng = 0; fng < 3; ++fng) {
                    std::deque<cv::Point> current_joint_point;

                    Vector current_joint_pixel;
                    itf_head_gaze->get2DPixel(camsel, Ha*(finger[fng].getH0().getCol(3)), current_joint_pixel);

                    current_joint_point.push_front(cv::Point(static_cast<int>(current_joint_pixel[0]), static_cast<int>(current_joint_pixel[1])));
                    cv::circle(img, current_joint_point.front(), 3, cv::Scalar(0, 0, 255), 4);

                    cv::line(img, endeffector_point, current_joint_point.front(), cv::Scalar(255, 0, 0), 2);

                    for (unsigned int i = 0; i < finger[fng].getN(); ++i) {
                        Vector current_joint_pixel;
                        itf_head_gaze->get2DPixel(camsel, Ha*(finger[fng].getH(i, true).getCol(3)), current_joint_pixel);

                        current_joint_point.push_front(cv::Point(static_cast<int>(current_joint_pixel[0]), static_cast<int>(current_joint_pixel[1])));
                        cv::circle(img, current_joint_point.front(), 3, cv::Scalar(0, 0, 255), 4);

                        cv::line(img, current_joint_point.front(), current_joint_point.back(), cv::Scalar(255, 255, 255), 2);
                        current_joint_point.pop_back();
                    }
                }

                Bottle &camPoseBottle = port_cam_pose.prepare();
                camPoseBottle.clear();
                camPoseBottle.addString(cam_x.toString() + "    " + cam_o.toString());

                outport_skeleton_img.write();
                port_cam_pose.write();
            }
        }
    }

    void onStop() {
        inport_skeleton_img.interrupt();
        outport_skeleton_img.interrupt();
        port_cam_pose.interrupt();
    }

    void threadRelease() {
        yInfo() << log_ID << "Deallocating resource of hand skeleton drawing thread.";

        if (!inport_skeleton_img.isClosed())  inport_skeleton_img.close();
        if (!outport_skeleton_img.isClosed()) outport_skeleton_img.close();
        if (!port_cam_pose.isClosed())        port_cam_pose.close();

        yInfo() << log_ID << "Deallocation completed!";
    }
};

enum MipMaps {
    NEAREST = 0,
    LINEAR = 1
};

class ThreadControllerSHC : public SuperimposeHandCADIDL
{
private:
    ConstString log_ID;

    bool mesh_back;
    bool mesh_wires;
    MipMaps mesh_mmaps;
protected:
    bool mesh_background(const bool status)
    {
        if (status && !mesh_back) {
            yInfo() << log_ID << "Enabling background of the mesh window.";

            mesh_back = true;

            return true;
        } else if (!status && mesh_back) {
            yInfo() << log_ID << "Disabling background of the mesh window.";

            mesh_back = false;

            return true;
        } else return false;
    }

    bool mesh_wireframe(const bool status)
    {
        if (status && !mesh_wires) {
            yInfo() << log_ID << "Enabling wireframe rendering.";

            mesh_wires = true;

            return true;
        } else if (!status && mesh_wires) {
            yInfo() << log_ID << "Disabling wireframe rendering.";

            mesh_wires = false;

            return true;
        } else return false;
    }

    bool mesh_mipmaps(const std::string& type)
    {
        if (type == "nearest") {
            yInfo() << log_ID << "Setting mipmaps color filtering to nearest neighbor.";

            mesh_mmaps = NEAREST;

            return true;
        } else if (type == "linear") {
            yInfo() << log_ID << "Setting mipmaps color filtering to linear.";

            mesh_mmaps = LINEAR;

            return true;
        } else return false;
    }
public:
    ThreadControllerSHC() : log_ID("[ThreadControllerSHC]"), mesh_back(true), mesh_wires(true), mesh_mmaps(NEAREST) {};

    bool getBackgroundOpt() { return mesh_back; }

    bool getWireframeOpt()  { return mesh_wires; }

    MipMaps getMipmapsOpt()    { return mesh_mmaps; }
};


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

    bool setCommandPort()
    {
        yInfo() << log_ID << "Opening command port.";
        if (!port_command.open("/"+PROJECT_NAME+"/cad/cmd")) {
            yError() << log_ID << "Cannot open the command port.";
            return false;
        }
        if (!helper.yarp().attachAsServer(port_command)) {
            yError() << log_ID << "Cannot attach the command port.";
            return false;
        }
        yInfo() << log_ID << "Command port succesfully opened and attached. Ready to start and recieve commands.";

        return true;
    }
    
public:
    SuperimposeHandCADThread(const ConstString &laterality, const ConstString &camera,
                             PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver,
                             const ConstString &shader_background_vert, const ConstString &shader_background_frag,
                             const ConstString &shader_model_vert, const ConstString &shader_model_frag,
                             const std::unordered_map<std::string, ConstString> &cad_hand) :
                                log_ID("[SuperimposeHandCADThread]"), laterality(laterality), camera(camera), arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver), shader_background_vert(shader_background_vert), shader_background_frag(shader_background_frag), shader_model_vert(shader_model_vert), shader_model_frag(shader_model_frag), cad_hand(cad_hand), camsel((camera == "left")? 0:1) { }
    
    bool threadInit() {
        yInfo() << log_ID << "Initializing hand skeleton drawing thread.";
        
        yInfo() << log_ID << "Setting interfaces";
        
        arm_cartesian_driver.view(itf_arm_cart);
        if (!itf_arm_cart) {
            yError() << log_ID << "Error getting ICartesianControl interface in thread.";
            return false;
        }
        
        gaze_driver.view(itf_head_gaze);
        if (!itf_head_gaze) {
            yError() << log_ID << "Error getting IGazeControl interface.";
            return false;
        }

        IControlLimits *itf_fingers_lim;
        arm_remote_driver.view(itf_fingers_lim);
        if (!itf_fingers_lim) {
            yError() << log_ID << "Error getting IControlLimits interface in thread.";
            return false;
        }

        arm_remote_driver.view(itf_arm_encoders);
        if (!itf_arm_encoders) {
            yError() << log_ID << "Error getting IEncoders interface.";
            return false;
        }
        itf_arm_encoders->getAxes(&num_arm_enc);
        
        yInfo() << log_ID << "Interfaces set!";
        
        yInfo() << log_ID << "Opening ports for CAD images.";
        
        if (!inport_renderer_img.open("/"+PROJECT_NAME+"/cad/cam/"+camera+":i")) {
            yError() << log_ID << "Cannot open input image port for "+camera+" camera.";
            return false;
        }

        if (!outport_renderer_img.open("/"+PROJECT_NAME+"/cad/cam/"+camera+":o")) {
            yError() << log_ID << "Cannot open output image port for "+camera+" camera.";
            return false;
        }
        
        yInfo() << log_ID << "CAD image ports succesfully opened!";
        
        yInfo() << log_ID << "Opening ports for "+camera+" camera pose.";
        
        if (!port_cam_pose.open("/"+PROJECT_NAME+"/cad/"+camera+"/pose:o")) {
            yError() << log_ID << "Cannot open "+camera+" camera pose output port.";
            return false;
        }
        
        yInfo() << log_ID << "Port for "+camera+" camera succesfully opened!";
        
        Bottle btl_cam_left_info;
        itf_head_gaze->getInfo(btl_cam_left_info);
        Bottle *cam_left_info = btl_cam_left_info.findGroup("camera_intrinsics_left").get(1).asList();
        yInfo() << log_ID << "Camera Info: [" + cam_left_info->toString() + "].";
        EYE_L_FX = static_cast<float>(cam_left_info->get(0).asDouble());
        EYE_L_CX = static_cast<float>(cam_left_info->get(2).asDouble());
        EYE_L_FY = static_cast<float>(cam_left_info->get(5).asDouble());
        EYE_L_CY = static_cast<float>(cam_left_info->get(6).asDouble());

        yInfo() << log_ID << "Setting joint bounds for the fingers.";

        finger[0] = iCubFinger(laterality+"_thumb");
        finger[1] = iCubFinger(laterality+"_index");
        finger[2] = iCubFinger(laterality+"_middle");

        std::deque<IControlLimits*> temp_lim;
        temp_lim.push_front(itf_fingers_lim);
        for (int i = 0; i < 3; ++i) {
            if (!finger[i].alignJointsBounds(temp_lim)) {
                yError() << log_ID << "Cannot set joint bound for finger " + std::to_string(i) + ".";
                return false;
            }
        }

        yInfo() << log_ID << "Joint bound for finger set!";

        yInfo() << log_ID << "Setting up OpenGL renderers.";
        /* Make the OpenGL context of window the current one handled by this thread. */
        glfwMakeContextCurrent(window);
        
        /* Create a background texture. */
        glGenTextures(1, &texture);

        /* Crate the squared support for the backround texture. */
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        GLfloat vertices[] = {//Positions      //Colors            //Texture Coords
                                1.0f,  1.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // Top Right
                                1.0f, -1.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // Bottom Right
                               -1.0f, -1.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // Bottom Left
                               -1.0f,  1.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // Top Left
        };
        
        GLuint indices[] = { 0, 1, 3,   // First Triangle
                             1, 2, 3 }; // Second Triangle
        
        /* Create and bind an element buffer object. */
        glGenBuffers(1, &ebo);

        glGenBuffers(1, &vbo);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
        
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));
        
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        
        glBindVertexArray(0);
        
        /* Crate shader program. */
        shader_background = new Shader(shader_background_vert.c_str(), shader_background_frag.c_str());
        shader_cad = new Shader(shader_model_vert.c_str(), shader_model_frag.c_str());
        
        /* Load models. */
        for (auto map = cad_hand.cbegin(); map != cad_hand.cend(); ++map) {
            yInfo() << log_ID << "Loading OpenGL "+map->first+" model.";
            hand_model[map->first] = new Model(map->second.c_str());
        }

        /* Predefined rotation matrices. */
        root_to_ogl = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
                                1.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f);

        back_proj = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, FAR*100.f);

        /* Projection matrix. */
        /* Intrinsic camera matrix: (232.921 0.0     162.202 0.0
                                     0.0     232.43  125.738 0.0
                                     0.0     0.0     1.0     0.0) */
        projection = glm::mat4(2.0f*EYE_L_FX/FRAME_WIDTH,       0,                                  0,                           0,
                               0,                               2.0f*EYE_L_FY/FRAME_HEIGHT,         0,                           0,
                               2.0f*(EYE_L_CX/FRAME_WIDTH)-1,   2.0f*(EYE_L_CY/FRAME_HEIGHT)-1,     -(FAR+NEAR)/(FAR-NEAR),     -1,
                               0,                               0,                                  -2.0f*FAR*NEAR/(FAR-NEAR),   0 );
        
        yInfo() << log_ID << "OpenGL renderers succesfully set up!";

        yInfo() << log_ID << "Setting up thread helper.";

        if (!setCommandPort()) return false;

        yInfo() << log_ID << "Thread helper succesfully set up!";

        yInfo() << log_ID << "Initialization completed!";
        
        return true;
    }
    
    void run() {
        Vector ee_x(3);
        Vector ee_o(4);
        Vector cam_x(3);
        Vector cam_o(4);
        while (!isStopping()) {
            
            ImageOf<PixelRgb> *imgin = inport_renderer_img.read(true);

            itf_arm_cart->getPose(ee_x, ee_o);

            itf_head_gaze->getLeftEyePose(cam_x, cam_o);

            Matrix Ha = axis2dcm(ee_o);
            ee_x.push_back(1.0);
            Ha.setCol(3, ee_x);

            Vector encs(static_cast<size_t>(num_arm_enc));
            Vector chainjoints;
            itf_arm_encoders->getEncoders(encs.data());
            for (unsigned int i = 0; i < 3; ++i) {
                finger[i].getChainJoints(encs, chainjoints);
                finger[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            //TODO: Improve fingers representation
            HandPose hand_pose;
            hand_pose["palm"] = {ee_x, ee_o};
            for (unsigned int fng = 0; fng < 3; ++fng) {

                std::string finger_s;
                Vector j_x;
                Vector j_o;

                if (fng != 0) {
                    j_x = (Ha * (finger[fng].getH0().getCol(3))).subVector(0, 2);
                    j_o = dcm2axis(Ha * finger[fng].getH0());

                    if      (fng == 1) { finger_s = "index0"; }
                    else if (fng == 2) { finger_s = "medium0"; }

                    hand_pose[finger_s] = {j_x, j_o};
                }

                for (unsigned int i = 0; i < finger[fng].getN(); ++i) {
                    j_x = (Ha * (finger[fng].getH(i, true).getCol(3))).subVector(0, 2);
                    j_o = dcm2axis(Ha * finger[fng].getH(i, true));

                    if      (fng == 0) { finger_s = "thumb"+std::to_string(i+1); }
                    else if (fng == 1) { finger_s = "index"+std::to_string(i+1); }
                    else if (fng == 2) { finger_s = "medium"+std::to_string(i+1); }

                    hand_pose[finger_s] = {j_x, j_o};
                }
            }

            if (imgin != NULL) {
                /* Load and generate the texture. */
                glBindTexture(GL_TEXTURE_2D, texture);

                /* Set the texture wrapping/filtering options (on the currently bound texture object). */
                if (helper.getMipmapsOpt() == NEAREST) {
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                } else if (helper.getMipmapsOpt() == LINEAR) {
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                }
                
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgin->width(), imgin->height(), 0, GL_RGB, GL_UNSIGNED_BYTE, imgin->getRawImage());
                glGenerateMipmap(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, 0);
                
                /* Clear the colorbuffer. */
                glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                
                /* Draw the background picture. */
                if (helper.getBackgroundOpt()) {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    shader_background->Use();
                    glUniformMatrix4fv(glGetUniformLocation(shader_background->Program, "projection"), 1, GL_FALSE, glm::value_ptr(back_proj));
                    glBindTexture(GL_TEXTURE_2D, texture);
                    glBindVertexArray(vao);
                    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
                    glBindVertexArray(0);
                }

                if (helper.getWireframeOpt()) {
                    /* Wireframe only. */
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                } else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                
                /* Use/Activate the shader. */
                shader_cad->Use();

                /* View transformation matrix. */
                /* Extrinsic camera matrix: */
                glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
                glm::mat4 root_eye_o = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));
                
                glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                             glm::mat3(root_to_ogl) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(root_eye_o) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                             glm::mat3(root_to_ogl) * glm::mat3(root_eye_o) * glm::vec3(0.0f, -1.0f, 0.0f));
                
                glUniformMatrix4fv(glGetUniformLocation(shader_cad->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
                
                glUniformMatrix4fv(glGetUniformLocation(shader_cad->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

                /* Model transformation matrix. */
                for (auto map = hand_pose.cbegin(); map != hand_pose.cend(); ++map) {
                    Vector j_x = hand_pose[map->first].first;
                    Vector j_o = hand_pose[map->first].second;

                    glm::mat4 root_j_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(j_x[0]), static_cast<float>(j_x[1]), static_cast<float>(j_x[2])));
                    glm::mat4 root_j_o = glm::rotate(glm::mat4(1.0f), static_cast<float>(j_o[3]), glm::vec3(static_cast<float>(j_o[0]), static_cast<float>(j_o[1]), static_cast<float>(j_o[2])));

                    glm::mat4 model = root_to_ogl * (root_j_t * root_j_o);

                    glUniformMatrix4fv(glGetUniformLocation(shader_cad->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

                    hand_model[map->first]->Draw(*shader_cad);
                }

                unsigned char *ogl_pixel = new unsigned char [3 * FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glReadPixels(0, 0, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, ogl_pixel);
                for (size_t i = 0; i < (FRAMEBUFFER_HEIGHT / 2); ++i) {
                    unsigned char(&row_bot)[3 * FRAMEBUFFER_WIDTH] = *reinterpret_cast<unsigned char(*)[3 * FRAMEBUFFER_WIDTH]>(&ogl_pixel[3 * FRAMEBUFFER_WIDTH * i]);
                    unsigned char(&row_up) [3 * FRAMEBUFFER_WIDTH] = *reinterpret_cast<unsigned char(*)[3 * FRAMEBUFFER_WIDTH]>(&ogl_pixel[3 * FRAMEBUFFER_WIDTH * (FRAMEBUFFER_HEIGHT-1 - i)]);
                    std::swap(row_bot, row_up);
                }

                /* Swap the buffers. */
                glfwSwapBuffers(window);

                ImageOf<PixelRgb> &imgout = outport_renderer_img.prepare();
                imgout.resize(FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);
                imgout.setExternal(ogl_pixel, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);
                delete [] ogl_pixel;
                
                Bottle &camPoseBottle = port_cam_pose.prepare();
                camPoseBottle.clear();
                camPoseBottle.addString(cam_x.toString() + "    " + cam_o.toString());

                outport_renderer_img.write();
                port_cam_pose.write();
            }
        }
    }

    void onStop() {
        inport_renderer_img.interrupt();
        outport_renderer_img.interrupt();
        port_cam_pose.interrupt();
    }

    void threadRelease() {
        yInfo() << log_ID << "Deallocating resource of renderer thread.";

        if (!inport_renderer_img.isClosed())  inport_renderer_img.close();
        if (!outport_renderer_img.isClosed()) outport_renderer_img.close();
        if (!port_cam_pose.isClosed())        port_cam_pose.close();

        yInfo() << log_ID << "Deleting OpenGL models and vertices.";
        for (auto map = hand_model.begin(); map != hand_model.end(); ++map) {
            yInfo() << log_ID << "Deleting OpenGL "+map->first+" model.";
            delete map->second;
        }
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &ebo);
        glDeleteBuffers(1, &vbo);
        glDeleteTextures(1, &texture);

        yInfo() << log_ID << "Deleting OpenGL shaders.";
        delete shader_background;
        delete shader_cad;

        yInfo() << log_ID << "Closing OpenGL context.";
        glfwMakeContextCurrent(NULL);

        yInfo() << log_ID << "Closing OpenGL window.";
        glfwSetWindowShouldClose(window, GL_TRUE);

        if (port_command.isOpen()) port_command.close();

        yInfo() << log_ID << "Deallocation completed!";
    }
};

class SuperimposeHand : public RFModule,
                        public SuperimposeHandIDL
{
private:
    const ConstString log_ID;

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

    bool fileFound (const ConstString &file) {
        if (file.empty()) {
            yError() << log_ID << "File not found!";
            return false;
        }
        return true;
    }

    bool setRightArmRemoteControlboard()
    {
        Property rightarm_remote_options;
        rightarm_remote_options.put("device", "remote_controlboard");
        rightarm_remote_options.put("local", "/"+PROJECT_NAME+"/control_right_arm");
        rightarm_remote_options.put("remote", "/"+robot+"/right_arm");

        rightarm_remote_driver.open(rightarm_remote_options);
        if (rightarm_remote_driver.isValid()) {
            yInfo() << log_ID << "Right arm remote_controlboard succefully opened.";

            rightarm_remote_driver.view(itf_rightarm_enc);
            if (!itf_rightarm_enc) {
                yError() << log_ID << "Error getting right arm IEncoders interface.\n";
                return false;
            }
            num_rightarm_enc = 0;
            itf_rightarm_enc->getAxes(&num_rightarm_enc);
            yInfo() << log_ID << "Right arm encorders succefully read.";

            rightarm_remote_driver.view(itf_rightarm_pos);
            if (!itf_rightarm_pos) {
                yError() << log_ID << "Error getting right arm IPositionControl2 interface.\n";
                return false;
            }
            yInfo() << log_ID << "Right arm positions succefully read.";
        } else {
            yError() << log_ID << "Error opening right arm remote_controlboard device.\n";
            return false;
        }

        num_rightarm_joint = 0;
        itf_rightarm_pos->getAxes(&num_rightarm_joint);
        yInfo() << log_ID << "Total number of right arm joints: " << num_rightarm_joint << ".";
        Vector tmp(static_cast<size_t>(num_rightarm_joint));
        for (int i = 0; i < num_rightarm_joint; ++i) {
            tmp[i] = 10.0;
        }
        if (!itf_rightarm_pos->setRefAccelerations(tmp.data()))
        {
            yError() << log_ID << "Error setting right arm joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < num_rightarm_joint; ++i) {
            tmp[i] = 15.0;
            if (!itf_rightarm_pos->setRefSpeed(i, tmp[i]))
            {
                yError() << log_ID << "Error setting right arm joint speeds.\n";
                return false;
            }
        }
        yInfo() << log_ID << "Right arm joint speeds and accelerations succesfully set.";

        return true;
    }

    bool setRightArmCartesianController()
    {
        Property rightarm_cartesian_options;
        rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
        rightarm_cartesian_options.put("local", "/"+PROJECT_NAME+"/cart_right_arm");
        rightarm_cartesian_options.put("remote", "/"+robot+"/cartesianController/right_arm");

        rightarm_cartesian_driver.open(rightarm_cartesian_options);
        if (rightarm_cartesian_driver.isValid()) {
            rightarm_cartesian_driver.view(itf_rightarm_cart);
            if (!itf_rightarm_cart) {
                yError() << log_ID << "Error getting ICartesianControl interface.\n";
                return false;
            }
            yInfo() << log_ID << "cartesiancontrollerclient succefully opened.";
        } else {
            yError() << log_ID << "Error opening cartesiancontrollerclient device.\n";
            return false;
        }

        return true;
    }

    bool setHeadRemoteControlboard()
    {
        Property head_option;
        head_option.put("device", "remote_controlboard");
        head_option.put("local", "/"+PROJECT_NAME+"/control_head");
        head_option.put("remote", "/"+robot+"/head");

        head_remote_driver.open(head_option);
        if (head_remote_driver.isValid()) {
            yInfo() << log_ID << "Head remote_controlboard succefully opened.";

            head_remote_driver.view(itf_head_pos);
            if (!itf_head_pos) {
                yError() << log_ID << "Error getting head IPositionControl interface.\n";
                return false;
            }
            yInfo() << log_ID << "Head positions succefully read.";
        } else {
            yError() << log_ID << "Error opening head remote_controlboard device.";
            return false;
        }

        num_head_joint = 0;
        itf_head_pos->getAxes(&num_head_joint);
        yInfo() << log_ID << "Total number of head joints: " << num_head_joint << ".";
        Vector tmp(static_cast<size_t>(num_head_joint));
        for (int i = 0; i < num_head_joint; ++i) {
            tmp[i] = 10.0;
        }
        if (!itf_head_pos->setRefAccelerations(tmp.data()))
        {
            yError() << log_ID << "Error setting head joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < num_head_joint; ++i) {
            tmp[i] = 15.0;
            if (!itf_head_pos->setRefSpeed(i, tmp[i]))
            {
                yError() << log_ID << "Error setting head joint speeds.\n";
                return false;
            }
        }
        yInfo() << log_ID << "Head joint speeds and accelerations set.";

        return true;
    }

    bool setGazeController()
    {
        Property gaze_option;
        gaze_option.put("device", "gazecontrollerclient");
        gaze_option.put("local", "/"+PROJECT_NAME+"/gaze");
        gaze_option.put("remote", "/iKinGazeCtrl");

        gaze_driver.open(gaze_option);
        if (gaze_driver.isValid()) {
            gaze_driver.view(itf_head_gaze);
            if (!itf_head_gaze) {
                std::cerr << "Error getting IGazeControl interface.\n";
                return false;
            }
        } else {
            std::cerr << "Gaze control device not available.\n";
            return false;
        }

        return true;
    }

    bool setTorsoDOF()
    {
        Vector curDOF;
        itf_rightarm_cart->getDOF(curDOF);
        yInfo() << log_ID << "Old DOF: [" + curDOF.toString(0) + "].";
        yInfo() << log_ID << "Setting iCub to use the DOF from the torso.";
        Vector newDOF(curDOF);
        newDOF[0] = 1;
        newDOF[1] = 2;
        newDOF[2] = 1;
        if (!itf_rightarm_cart->setDOF(newDOF, curDOF)) {
            yError() << log_ID << "Cannot use torso DOF.";
            return false;
        }
        yInfo() << log_ID << "Setting the DOF done.";
        yInfo() << log_ID << "New DOF: [" + curDOF.toString(0) + "]";

        return true;
    }

    bool setCommandPort()
    {
        yInfo() << log_ID << "Opening command port.";
        if (!port_command.open("/"+PROJECT_NAME+"/cmd")) {
            yError() << log_ID << "Cannot open the command port.";
            return false;
        }
        if (!this->yarp().attachAsServer(port_command)) {
            yError() << log_ID << "Cannot attach the command port.";
            return false;
        }
        yInfo() << log_ID << "Command port succesfully opened and attached. Ready to start and recieve commands.";

        return true;
    }

    bool moveFingers(const double (&joint)[6])
    {
        /* Close iCub hand. */
        yInfo() << log_ID << "Closing fingers.";
        Vector rightarm_encoder(static_cast<size_t>(num_rightarm_enc));
        itf_rightarm_enc->getEncoders(rightarm_encoder.data());
        std::list<std::pair<unsigned int, double>> joint_pos_map = {{13, joint[0]},
                                                                    {14, joint[1]},
                                                                    {15, joint[2]},
                                                                    { 8, joint[3]},
                                                                    { 9, joint[4]},
                                                                    {10, joint[5]}};
        for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map) {
            yInfo() << log_ID << "Moving joint "+std::to_string(map->first)+" to the position "+std::to_string(map->second)+".";
            if (std::abs(rightarm_encoder[map->first] - map->second) > 5.0) {
                rightarm_encoder[map->first] = map->second;
                itf_rightarm_pos->positionMove(rightarm_encoder.data());
                Time::delay(2.0);
            }
        }
        yInfo() << log_ID << "Fingers succesfully closed.";

        return true;
    }

    bool moveHand(const Matrix &R, const Vector &init_x)
    {
        /* Setting hand pose */
        yInfo() << log_ID << "Moving hand to the initial position.";

        Vector init_o(dcm2axis(R));

        itf_rightarm_cart->goToPoseSync(init_x, init_o);
        itf_rightarm_cart->waitMotionDone(0.1, 6.0);

        yInfo() << log_ID << "The hand is in position.";

        /* Set initial fixation point */
        Vector tmp;
        itf_head_gaze->getFixationPoint(tmp);
        Vector init_fixation(init_x);
        init_fixation[0] -= 0.05;
        init_fixation[1] -= 0.05;
        if (norm(tmp - init_fixation) > 0.10) {
            yInfo() << log_ID << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
            itf_head_gaze->lookAtFixationPoint(init_fixation);
            itf_head_gaze->waitMotionDone(0.1, 6.0);
        }
        yInfo() << log_ID << "Gaze motion done.";

        return true;
    }

protected:
    bool move_hand()
    {
        if (!init_position) {
            yInfo() << log_ID << "Starting single hand motion.";

            start = true;

            return true;
        } else {
            yWarning() << log_ID << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

            return false;
        }
    }

    bool move_hand_freerun()
    {
        if (!init_position) {
            yInfo() << log_ID << "Starting freerun hand motion.";

            start = true;
            freerunning = true;

            return true;
        } else {
            yWarning() << log_ID << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

            return false;
        }
    }

    bool stop_hand()
    {
        yInfo() << log_ID << "Stopping hand motion when reaching the initial position.";

        start = false;
        if (freerunning) freerunning = false;

        return true;
    }

    bool initial_position()
    {
        if (!init_position) {
            yWarning() << log_ID << "Already in initial position settings!";

            return false;
        } else {
            yInfo() << log_ID << "Reaching initial position...";

            init_position = !moveHand(table_view_R, table_view_x);
            if (!init_position) yInfo() << log_ID << "...done. iCub can move the hand in this settings.";
            else yWarning() << log_ID << "...could not reach initial position!";

            return init_position;
        }
    }

    bool view_hand()
    {
        if (!start) {
            yInfo() << log_ID << "Reaching a position close to iCub left camera with the right hand...";

            init_position = moveHand(frontal_view_R, frontal_view_x);
            if (!init_position) yWarning() << log_ID << "...could not reach the desired position!";
            else yInfo() << log_ID << "...done. iCub can't move the hand in this settings.";

            return init_position;
        } else {
            yWarning() << log_ID << "Can't move hand while moving it!";

            return false;
        }
    }

    bool open_fingers()
    {
        yInfo() << log_ID << "Opening fingers...";

        bool motion_done = moveFingers(open_hand_joints);
        if (!motion_done) yWarning() << log_ID << "...fingers could not be opened!";
        else yInfo() << log_ID << "...done.";

        return motion_done;
    }

    bool close_fingers()
    {
        yInfo() << log_ID << "Closing fingers...";

        bool motion_done = moveFingers(closed_hand_joints);
        if (!motion_done) yWarning() << log_ID << "...fingers could not be closed!";
        else yInfo() << log_ID << "...done.";

        return motion_done;
    }

    bool view_skeleton(const bool status)
    {
        if (!superimpose_skeleton && status) {
            trd_left_cam_skeleton = new SuperimposeHandSkeletonThread("right", "left", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver);

            if (trd_left_cam_skeleton != NULL) {
                yInfo() << log_ID << "Starting skeleton superimposing thread for the right hand on the left camera images...";

                if (!trd_left_cam_skeleton->start()) {
                    yWarning() << log_ID << "...thread could not be started!";

                    superimpose_skeleton = false;
                } else {
                    yInfo() << log_ID << "...done.";

                    superimpose_skeleton = true;
                }
            } else {
                yWarning() << log_ID << "Could not initialize hand skeleton superimposition!";

                superimpose_skeleton = false;
            }

            return superimpose_skeleton;

        } else if (superimpose_skeleton && !status) {
            yInfo() << log_ID << "Stopping hand skeleton superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_skeleton->stop()) {
                yWarning() << log_ID << "...thread could not be stopped!";

                superimpose_skeleton = true;
            } else {
                yInfo() << log_ID << "...done.";

                delete trd_left_cam_skeleton;
                trd_left_cam_cad = nullptr;

                superimpose_skeleton = false;
            }

            return !superimpose_skeleton;

        } else return false;
    }

    bool view_mesh(const bool status) {
        if (!superimpose_mesh && status) {
            trd_left_cam_cad = new SuperimposeHandCADThread("right", "left", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver, shader_background_vert, shader_background_frag, shader_model_vert, shader_model_frag, cad_hand);

            if (trd_left_cam_cad != NULL) {
                yInfo() << log_ID << "Starting mesh superimposing thread for the right hand on the left camera images...";

                if (!trd_left_cam_cad->start()) {
                    yWarning() << log_ID << "...thread could not be started!";

                    superimpose_mesh = false;
                } else {
                    yInfo() << log_ID << "...done.";

                    superimpose_mesh = true;
                }
            } else {
                yWarning() << log_ID << "Could not initialize hand mesh superimposition!";

                superimpose_mesh = false;
            }

            return superimpose_mesh;

        } else if (superimpose_mesh && !status) {
            yInfo() << log_ID << "Stopping hand mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_cad->stop()) {
                yWarning() << log_ID << "...thread could not be stopped!";

                superimpose_mesh = true;
            } else {
                yInfo() << log_ID << "...done.";

                delete trd_left_cam_cad;
                trd_left_cam_cad = nullptr;
                
                superimpose_mesh = false;
            }

            return !superimpose_mesh;

        } else return false;
    }

    std::string quit() {
        yInfo() << log_ID << "Quitting...";

        this->stopModule();

        return "[bye]";
    }

public:
    SuperimposeHand() : log_ID("[SuperimposeHand]") {}

    double getPeriod() { return 0.0; }

    bool configure(ResourceFinder &rf)
    {
        this->setName(PROJECT_NAME.c_str());

        /* Setting default parameters. */
        start = false;
        init_position = false;
        freerunning = false;
        superimpose_skeleton = false;
        superimpose_mesh = false;

        /* Parsing parameters from config file. */
        robot = rf.findGroup("PARAMETER").check("robot", Value("icub")).asString();
        if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16) {
            Vector arm_vel(16);
            for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i) {
                arm_vel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
            }
            yInfo() << log_ID << arm_vel.toString();
        }

        /* Looking for OpenGL shaders and CAD files. */
        shader_background_vert = rf.findFileByName("shader_background.vert");
        if (!fileFound(shader_background_vert)) return false;

        shader_background_frag = rf.findFileByName("shader_background.frag");
        if (!fileFound(shader_background_frag)) return false;

        shader_model_vert = rf.findFileByName("shader_model.vert");
        if (!fileFound(shader_model_vert)) return false;

        shader_model_frag = rf.findFileByName("shader_model_simple.frag");
        if (!fileFound(shader_model_frag)) return false;

        //TODO: use simplified dae instead of obj
        cad_hand["palm"] = rf.findFileByName("r_palm.obj");
        if (!fileFound(cad_hand["palm"])) return false;
        cad_hand["thumb1"] = rf.findFileByName("r_tl0.obj");
        if (!fileFound(cad_hand["thumb1"])) return false;
        cad_hand["thumb2"] = rf.findFileByName("r_tl1.obj");
        if (!fileFound(cad_hand["thumb2"])) return false;
        cad_hand["thumb3"] = rf.findFileByName("r_tl2.obj");
        if (!fileFound(cad_hand["thumb3"])) return false;
        cad_hand["thumb4"] = rf.findFileByName("r_tl3.obj");
        if (!fileFound(cad_hand["thumb4"])) return false;
        cad_hand["thumb5"] = rf.findFileByName("r_tl4.obj");
        if (!fileFound(cad_hand["thumb5"])) return false;
        cad_hand["index0"] = rf.findFileByName("r_indexbase.obj");
        if (!fileFound(cad_hand["index0"])) return false;
        cad_hand["index1"] = rf.findFileByName("r_ail0.obj");
        if (!fileFound(cad_hand["index1"])) return false;
        cad_hand["index2"] = rf.findFileByName("r_ail1.obj");
        if (!fileFound(cad_hand["index2"])) return false;
        cad_hand["index3"] = rf.findFileByName("r_ail2.obj");
        if (!fileFound(cad_hand["index3"])) return false;
        cad_hand["index4"] = rf.findFileByName("r_ail3.obj");
        if (!fileFound(cad_hand["index4"])) return false;
        cad_hand["medium0"] = rf.findFileByName("r_ml0.obj");
        if (!fileFound(cad_hand["medium0"])) return false;
        cad_hand["medium1"] = rf.findFileByName("r_ml1.obj");
        if (!fileFound(cad_hand["medium1"])) return false;
        cad_hand["medium2"] = rf.findFileByName("r_ml2.obj");
        if (!fileFound(cad_hand["medium2"])) return false;
        cad_hand["medium3"] = rf.findFileByName("r_ml3.obj");
        if (!fileFound(cad_hand["medium3"])) return false;


        /* Initializing useful pose matrices and vectors for the hand. */
        frontal_view_R.resize(3, 3);
        frontal_view_R(0,0) =  0.0;   frontal_view_R(0,1) =  0.0;   frontal_view_R(0,2) =  1.0;
        frontal_view_R(1,0) = -1.0;   frontal_view_R(1,1) =  0.0;   frontal_view_R(1,2) =  0.0;
        frontal_view_R(2,0) =  0.0;   frontal_view_R(2,1) = -1.0;   frontal_view_R(2,2) =  0.0;

        frontal_view_x.resize(3);
        frontal_view_x[0] = -0.25;
        frontal_view_x[1] = +0.00;
        frontal_view_x[2] = +0.20;

        table_view_R.resize(3, 3);
        table_view_R(0,0) = -1.0;   table_view_R(0,1) =  0.0;   table_view_R(0,2) =  0.0;
        table_view_R(1,0) =  0.0;   table_view_R(1,1) =  1.0;   table_view_R(1,2) =  0.0;
        table_view_R(2,0) =  0.0;   table_view_R(2,1) =  0.0;   table_view_R(2,2) = -1.0;

        table_view_x.resize(3);
        table_view_x[0] = -0.40;
        table_view_x[1] = +0.10;
        table_view_x[2] = +0.10;

        open_hand_joints[0] = 0;
        open_hand_joints[1] = 0;
        open_hand_joints[2] = 0;
        open_hand_joints[3] = 10;
        open_hand_joints[4] = 0;
        open_hand_joints[5] = 0;

        closed_hand_joints[0] = 80;
        closed_hand_joints[1] = 150;
        closed_hand_joints[2] = 180;
        closed_hand_joints[3] = 80;
        closed_hand_joints[4] = 10;
        closed_hand_joints[5] = 80;

        /* Right arm control board. */
        if (!setRightArmRemoteControlboard()) return false;

        /* Right arm cartesian controler. */
        if (!setRightArmCartesianController()) return false;

        /* Head control board. */
        if (!setHeadRemoteControlboard()) return false;

        /* Gaze control. */
        if (!setGazeController()) return false;

        /* Enable torso DOF. */
        if (!setTorsoDOF()) return false;

        /* Set default right hand configuration (closed). */
        if (!moveFingers(open_hand_joints)) return false;

        /* Set deafult initial pose of the hand (table view). */
        if (!moveHand(table_view_R, table_view_x)) return false;

        /* Set initial finger motion point */
        radius = 0.08;
        angle_ratio = 12;
        motion_time = 10.0;
        path_time = motion_time / angle_ratio;
        itf_rightarm_cart->setTrajTime(path_time);

        /* Open a remote command port and allow the program be started */
        return setCommandPort();

    }

    bool updateModule()
    {
        if (start) {
            Vector motion_axis;
            Vector motion_angle;
            itf_rightarm_cart->getPose(motion_axis, motion_angle);
            Vector center(2);
            center[0] = motion_axis[0];
            center[1] = motion_axis[1] - radius;

            yInfo() << log_ID << "Starting finger motion.";
            for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angle_ratio) {
                motion_axis[0] = (center[0] - (radius * sin(alpha)));
                motion_axis[1] = (center[1] + (radius * cos(alpha)));
                yInfo() << log_ID << "Next position: [" << motion_axis.toString() << "].";
                itf_rightarm_cart->goToPose(motion_axis, motion_angle);
                Time::delay(0.7 * path_time);
            }
            yInfo() << log_ID << "Motion done.";
            if (!freerunning) start = false;
        }
        return true;
    }

    bool interruptModule()
    {
        if (superimpose_skeleton) {
            trd_left_cam_skeleton->stop();
            trd_left_cam_skeleton = nullptr;
        }

        if (superimpose_mesh) {
            trd_left_cam_cad->stop();
            trd_left_cam_cad = nullptr;
        }

        return true;
    }

    bool close()
    {
        yInfo() << log_ID << "Calling close functions...";

        delete trd_left_cam_skeleton;
        delete trd_left_cam_cad;

        itf_rightarm_cart->removeTipFrame();

        if (rightarm_cartesian_driver.isValid()) rightarm_cartesian_driver.close();
        if (rightarm_remote_driver.isValid()) rightarm_remote_driver.close();
        if (head_remote_driver.isValid()) head_remote_driver.close();
        if (gaze_driver.isValid()) gaze_driver.close();

        if (port_command.isOpen()) port_command.close();
        return true;
    }

};

class SuperimposeHandThread : public Thread
{
private:
    const ConstString log_ID;
    SuperimposeHand &sh;
    ResourceFinder &rf;
public:
    SuperimposeHandThread(SuperimposeHand &sh, ResourceFinder &rf) : log_ID("[SuperimposeHandThread]"), sh(sh), rf(rf) {}
    
    bool threadInit() {
        if (!sh.configure(rf)) {
            yError() << log_ID << "RFModule failed to open.";
            return false;
        }
        yInfo() << log_ID << "RFModule succesfully opened.";
        return true;
    }
    
    void run() {
        sh.runModule();
    }
    
    void threadRelease() {
        yInfo() << log_ID << "Releasing.";
    }
};

bool fileFound(const ConstString &file);

int main(int argc, char *argv[])
{
    ConstString log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    Network yarp;
    if (!yarp.checkNetwork(3.0)) {
        yError() << log_ID << "YARP seems unavailable.";
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("superimpose-hand_config.ini");
    rf.setDefaultContext("superimpose-hand");
    rf.configure(argc, argv);

    yInfo() << log_ID << "Setting up OpenGL.";
    
    /* Initialize GLFW. */
    if (glfwInit() == GL_FALSE) {
        yError() << log_ID << "Failed to initialize GLFW.";
        return false;
    }
    
    /* Set context properties by "hinting" specific (property, value) pairs. */
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
#ifdef GLFW_MAC
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    
    /* Create a window. */
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL Window", nullptr, nullptr);
    if (window == nullptr) {
        yError() << log_ID << "Failed to create GLFW window.";
        glfwTerminate();
        return false;
    }
    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window);
    
    /* Initialize GLEW to use the OpenGL implementation provided by the videocard manufacturer. */
    /* Note: remember that the OpenGL are only specifications, the implementation is provided by the manufacturers. */
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        yError() << log_ID << "Failed to initialize GLEW.";
        return false;
    }
    
    /* Set OpenGL rendering frame for the current window. */
    /* Note that the real monitor width and height may differ w.r.t. the choosen one in hdpi monitors. */
    int hdpi_width;
    int hdpi_height;
    glfwGetFramebufferSize(window, &hdpi_width, &hdpi_height);
    glViewport(0, 0, hdpi_width, hdpi_height);
    yInfo() << log_ID << "OpenGL viewport set to "+std::to_string(hdpi_width)+"x"+std::to_string(hdpi_height)+".";
    
    /* Set GL property. */
    glEnable(GL_DEPTH_TEST);

    glfwPollEvents();

    yInfo() << log_ID << "OpenGL succesfully set up.";

    /* SuperimposeHand, derived from RFModule, must be declared by the main thread (thread_0). */
    SuperimposeHand sh;
    SuperimposeHandThread trd_sh(sh, rf);

    trd_sh.start();
    while (trd_sh.isRunning()) {
        glfwPollEvents();
    }
    
    glfwMakeContextCurrent(NULL);
    glfwTerminate();

    yInfo() << log_ID << "Main returning.";
    yInfo() << log_ID << "Application closed.";
    return 0;
}
