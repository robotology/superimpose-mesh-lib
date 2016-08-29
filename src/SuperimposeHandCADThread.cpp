#include "SuperimposeHandCADThread.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <iCub/ctrl/math.h>

#define PROJECT_NAME yarp::os::ConstString("superimpose_hand")
#define WINDOW_WIDTH  320
#define WINDOW_HEIGHT 240
#ifdef GLFW_RETINA
#define FRAMEBUFFER_WIDTH  2*WINDOW_WIDTH
#define FRAMEBUFFER_HEIGHT 2*WINDOW_HEIGHT
#else
#define FRAMEBUFFER_WIDTH  WINDOW_WIDTH
#define FRAMEBUFFER_HEIGHT WINDOW_HEIGHT
#endif
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define NEAR 0.001f
#define FAR  1000.0f

using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


bool SuperimposeHandCADThread::setCommandPort()
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


SuperimposeHandCADThread::SuperimposeHandCADThread(const ConstString &laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver, const ConstString &shader_background_vert, const ConstString &shader_background_frag, const ConstString &shader_model_vert, const ConstString &shader_model_frag, const std::unordered_map<std::string, ConstString> &cad_hand, GLFWwindow *window) :
        log_ID("[SuperimposeHandCADThread]"), laterality(laterality), camera(camera), arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver), shader_background_vert(shader_background_vert), shader_background_frag(shader_background_frag), shader_model_vert(shader_model_vert), shader_model_frag(shader_model_frag), cad_hand(cad_hand), camsel((camera == "left")? 0:1), window(window) { }


bool SuperimposeHandCADThread::threadInit() {
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


void SuperimposeHandCADThread::run() {
    Vector ee_x(3);
    Vector ee_o(4);
    Vector cam_x(3);
    Vector cam_o(4);
    unsigned char *ogl_pixel = new unsigned char [3 * FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];

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

            Bottle &camPoseBottle = port_cam_pose.prepare();
            camPoseBottle.clear();
            camPoseBottle.addString(cam_x.toString() + "    " + cam_o.toString());

            outport_renderer_img.write();
            port_cam_pose.write();

        }
    }
    delete [] ogl_pixel;
}


void SuperimposeHandCADThread::onStop() {
    inport_renderer_img.interrupt();
}


void SuperimposeHandCADThread::threadRelease() {
    yInfo() << log_ID << "Deallocating resource of renderer thread.";

    outport_renderer_img.interrupt();
    port_cam_pose.interrupt();

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
