#include "SuperImpose/SICAD.h"

#include <iostream>
#include <exception>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>


SICAD::SICAD(const ObjFileMap& objfile_map, const int cam_width, const int cam_height, const float eye_fx, const float eye_fy, const float eye_cx, const float eye_cy) :
    log_ID_("[SICAD]")
{
    if (!can_init) throw std::runtime_error("Can't create object SICAD before calling static function member SICAD::initOGL.");

    std::cout << log_ID_ << "Setting up OpenGL renderers." << std::endl;

    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);

    /* Enable scissor test. */
    glEnable(GL_SCISSOR_TEST);

    /* Create a background texture. */
    glGenTextures(1, &texture_);

    /* Crate the squared support for the backround texture. */
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    GLfloat vertices[] = {// Positions    // Colors            // Texture Coords
                             1.0f,  1.0f,    1.0f, 0.0f, 0.0f,    1.0f, 1.0f,   // Top Right
                             1.0f, -1.0f,    0.0f, 1.0f, 0.0f,    1.0f, 0.0f,   // Bottom Right
                            -1.0f, -1.0f,    0.0f, 0.0f, 1.0f,    0.0f, 0.0f,   // Bottom Left
                            -1.0f,  1.0f,    1.0f, 1.0f, 0.0f,    0.0f, 1.0f }; // Top Left

    GLuint indices[] = { 0, 1, 3,   // First Triangle
                         1, 2, 3 }; // Second Triangle

    /* Create and bind an element buffer object. */
    glGenBuffers(1, &ebo_);

    glGenBuffers(1, &vbo_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(0));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    /* Crate shader program. */
    shader_background_ = new (std::nothrow) Shader("shader_background.vert", "shader_background.frag");
    if (shader_background_ == nullptr) throw std::runtime_error("Runtime error: shader_background files not found!");

    shader_cad_ = new (std::nothrow) Shader("shader_model.vert", "shader_model_simple.frag");
    if (shader_cad_ == nullptr) throw std::runtime_error("Runtime error: shader_model files not found!");

    /* Load models. */
    for (auto map = objfile_map.cbegin(); map != objfile_map.cend(); ++map)
    {
        std::cout << log_ID_ << "Loading OpenGL "+map->first+" model." << std::endl;
        model_obj_[map->first] = new (std::nothrow) Model(map->second.c_str());
        if (model_obj_[map->first] == nullptr) throw std::runtime_error("Runtime error: file "+map->second+" not found!");
    }

    /* Predefined rotation matrices. */
    root_to_ogl_ = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
                             1.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 1.0f);

    back_proj_ = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, far_*100.f);

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921      0.0     162.202    0.0
                                   0.0      232.43    125.738    0.0
                                   0.0        0.0       1.0      0.0) */
    projection_ = glm::mat4(2.0f*(eye_fx/cam_width),    0,                              0,                                  0,
                            0,                          2.0f*(eye_fy/cam_height),       0,                                  0,
                            2.0f*(eye_cx/cam_width)-1,  2.0f*(eye_cy/cam_height)-1,    -(far_+near_)/(far_-near_),         -1,
                            0,                          0,                             -2.0f*(far_*near_)/(far_-near_),     0);

    /* Projection transformation matrix. */
    shader_cad_->Use();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));

    std::cout << log_ID_ << "OpenGL renderers succesfully set up!" << std::endl;

    std::cout << log_ID_ << "Initialization completed!" << std::endl;
}


SICAD::~SICAD()
{
    std::cout << log_ID_ << "Deallocating OpenGL resources..." << std::endl;

    glfwMakeContextCurrent(window_);

    for (auto map = model_obj_.begin(); map != model_obj_.end(); ++map)
    {
        std::cout << log_ID_ << "Deleting OpenGL "+map->first+" model." << std::endl;
        delete map->second;
    }

    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers     (1, &ebo_);
    glDeleteBuffers     (1, &vbo_);
    glDeleteTextures    (1, &texture_);

    std::cout << log_ID_ << "Deleting OpenGL shaders." << std::endl;
    delete shader_background_;
    delete shader_cad_;

    std::cout << log_ID_ << "Closing OpenGL context." << std::endl;
    glfwMakeContextCurrent(NULL);

    std::cout << log_ID_ << "Closing OpenGL window." << std::endl;
    glfwSetWindowShouldClose(window_, GL_TRUE);

    std::cout << log_ID_ << "OpenGL resource deallocation completed!" << std::endl;
}


bool SICAD::initOGL(const GLsizei width, const GLsizei height, const GLint viewports)
{
    std::string log_ID = "[OpenGL]";

    if (can_init)
    {
        std::cout << log_ID << "Already set up!" << std::endl;
        return false;
    }
    std::cout << log_ID << "Start setting up..." << std::endl;

    /* Initialize GLFW. */
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << log_ID << "Failed to initialize GLFW.";
        return false;
    }

    /* Set context properties by "hinting" specific (property, value) pairs. */
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE,        GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE,             GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE,               GL_FALSE);
#ifdef GLFW_MAC
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    /* Create a window. */
    viewports_ = viewports;
    window_ = glfwCreateWindow(width * viewports_, height, "OpenGL Window", nullptr, nullptr);
    if (window_ == nullptr)
    {
        std::cerr << log_ID << "Failed to create GLFW window.";
        glfwTerminate();
        return false;
    }
    glfwGetWindowSize(window_, &window_width_, &window_height_);
    std::cout << log_ID << "Window created with "+std::to_string(window_width_)+"x"+std::to_string(window_height_)+" size." << std::endl;

    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);

    /* Set window callback functions. */
    glfwSetKeyCallback(window_, key_callback);

    /* Initialize GLEW to use the OpenGL implementation provided by the videocard manufacturer. */
    /* Note: remember that the OpenGL are only specifications, the implementation is provided by the manufacturers. */
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cerr << log_ID << "Failed to initialize GLEW.";
        return false;
    }

    /* Set default OpenGL viewport for the current window. */
    /* Note that framebuffer_width_ and framebuffer_height_ may differ w.r.t. width and height in hdpi monitors. */
    glfwGetFramebufferSize(window_, &framebuffer_width_, &framebuffer_height_);
    glViewport(0, 0, framebuffer_width_, framebuffer_height_);
    std::cout << log_ID << "The window framebuffer is "+std::to_string(framebuffer_width_)+"x"+std::to_string(framebuffer_height_)+"." << std::endl;

    /* Set GL property. */
    glEnable(GL_DEPTH_TEST);

    glfwPollEvents();

    can_init = true;
    std::cout << log_ID << "Succesfully set up!" << std::endl;

    return true;
}


int SICAD::oglWindowShouldClose()
{
    return glfwWindowShouldClose(window_);
}


bool SICAD::superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    glfwMakeContextCurrent(window_);

    glViewport(0, 0, framebuffer_width_ / viewports_, framebuffer_height_);
    glScissor (0, 0, framebuffer_width_ / viewports_, framebuffer_height_);

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt()) set_background(img);

    /* View mesh filled or as wireframe. */
    set_wireframe(getWireframeOpt());

    /* Use/Activate the shader. */
    shader_cad_->Use();

    /* View transformation matrix. */
    glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 eye_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl_) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(root_to_ogl_) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(eye_to_root) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(root_to_ogl_) * glm::mat3(eye_to_root) * glm::vec3(0.0f, -1.0f, 0.0f));

    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    /* Model transformation matrix. */
    for (auto map = objpos_map.cbegin(); map != objpos_map.cend(); ++map)
    {
        const double * pose = map->second.data();

        glm::mat4 obj_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        obj_to_root[3][0] = pose[0];
        obj_to_root[3][1] = pose[1];
        obj_to_root[3][2] = pose[2];

        glm::mat4 model = root_to_ogl_ * obj_to_root;

        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

        model_obj_[map->first]->Draw(*shader_cad_);
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_, framebuffer_width_ / viewports_, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, 0, framebuffer_width_ / viewports_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, ogl_pixel, 0);
    cv::resize(ogl_pixel, img, cv::Size(window_width_ / viewports_, window_height_), 0, 0, cv::INTER_LINEAR);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    return true;
}


bool SICAD::superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    glfwMakeContextCurrent(window_);

    shader_cad_->Use();

    /* View transformation matrix. */
    glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 eye_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl_) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(root_to_ogl_) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(eye_to_root) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(root_to_ogl_) * glm::mat3(eye_to_root) * glm::vec3(0.0f, -1.0f, 0.0f));

    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    /* Model transformation matrix. */
    const int num_img = objpos_multimap.size();
    for (unsigned int i = 0; i < num_img; ++i)
    {
        glViewport((framebuffer_width_ / num_img) * i, 0,
                   (framebuffer_width_ / num_img)    , framebuffer_height_);
        glScissor ((framebuffer_width_ / num_img) * i, 0,
                   (framebuffer_width_ / num_img)    , framebuffer_height_);

        /* Clear the colorbuffer. */
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* Draw the background picture. */
        if (getBackgroundOpt()) set_background(img, i);

        /* View mesh filled or as wireframe. */
        set_wireframe(getWireframeOpt());

        /* Use/Activate the shader. */
        shader_cad_->Use();

        for (auto map = objpos_multimap[i].cbegin(); map != objpos_multimap[i].cend(); ++map)
        {
            const double * pose = map->second.data();

            glm::mat4 obj_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
            obj_to_root[3][0] = pose[0];
            obj_to_root[3][1] = pose[1];
            obj_to_root[3][2] = pose[2];

            glm::mat4 model = root_to_ogl_ * obj_to_root;

            glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

            model_obj_[map->first]->Draw(*shader_cad_);
        }
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
     and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_, framebuffer_width_, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, ogl_pixel, 0);
    cv::resize(ogl_pixel, img, cv::Size(window_width_, window_height_), 0, 0, cv::INTER_LINEAR);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    return true;
}


void SICAD::setBackgroundOpt(bool show_background)
{
    show_background_ = show_background;
}


bool SICAD::getBackgroundOpt() const
{
    return show_background_;
}


void SICAD::setWireframeOpt(bool show_mesh_wires)
{
    if  (show_mesh_wires) show_mesh_mode_ = GL_LINE;
    else                  show_mesh_mode_ = GL_FILL;
}


GLenum SICAD::getWireframeOpt() const
{
    return show_mesh_mode_;
}


SICAD::MipMaps SICAD::getMipmapsOpt() const
{
    return mesh_mmaps_;
}


void SICAD::set_background(cv::Mat& img, const unsigned int unit)
{
    /* Load and generate the texture. */
    glBindTexture(GL_TEXTURE_2D, texture_);

    /* Set the texture wrapping/filtering options (on the currently bound texture object). */
    if (getMipmapsOpt() == NEAREST)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    else if (getMipmapsOpt() == LINEAR)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    shader_background_->Use();
    glUniformMatrix4fv(glGetUniformLocation(shader_background_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(back_proj_));
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}


void SICAD::set_wireframe(GLenum mode)
{
    glPolygonMode(GL_FRONT_AND_BACK, mode);
}


void SICAD::key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    /* When a user presses the escape key, we set the WindowShouldClose property to true, closing the application. */
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) glfwSetWindowShouldClose(window, GL_TRUE);
}


bool        SICAD::can_init            = false;
GLFWwindow* SICAD::window_             = nullptr;
GLint       SICAD::viewports_          = 1;
GLsizei     SICAD::window_width_       = 0;
GLsizei     SICAD::window_height_      = 0;
GLsizei     SICAD::framebuffer_width_  = 0;
GLsizei     SICAD::framebuffer_height_ = 0;
