#include "SICAD.h"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#define WINDOW_WIDTH       320
#define WINDOW_HEIGHT      240
#ifdef GLFW_RETINA
#define FRAMEBUFFER_WIDTH  2*WINDOW_WIDTH
#define FRAMEBUFFER_HEIGHT 2*WINDOW_HEIGHT
#else
#define FRAMEBUFFER_WIDTH  WINDOW_WIDTH
#define FRAMEBUFFER_HEIGHT WINDOW_HEIGHT
#endif
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define NEAR         0.001f
#define FAR          1000.0f


SICAD::SICAD() : log_ID_("[SH-CAD]")
{
    show_background_ = false;
    mesh_wires_      = false;
    mesh_mmaps_      = NEAREST;
}


SICAD::~SICAD() {
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


bool SICAD::Configure(GLFWwindow *& window, const ObjFileMap & obj2fil_map, const float EYE_FX, const float EYE_FY, const float EYE_CX, const float EYE_CY)
{
    std::cout << log_ID_ << "Setting up OpenGL renderers." << std::endl;

    window_ = window;

    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);

    /* Create a background texture. */
    glGenTextures(1, &texture_);

    /* Crate the squared support for the backround texture. */
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    GLfloat vertices[] = {// Positions    // Colors            // Texture Coords
                             1.0f,  1.0f,    1.0f, 0.0f, 0.0f,    1.0f, 1.0f,   // Top Right
                             1.0f, -1.0f,    0.0f, 1.0f, 0.0f,    1.0f, 0.0f,   // Bottom Right
                            -1.0f, -1.0f,    0.0f, 0.0f, 1.0f,    0.0f, 0.0f,   // Bottom Left
                            -1.0f,  1.0f,    1.0f, 1.0f, 0.0f,    0.0f, 1.0f    // Top Left
                         };

    GLuint indices[] = { 0, 1, 3,   // First Triangle
                         1, 2, 3 }; // Second Triangle

    /* Create and bind an element buffer object. */
    glGenBuffers(1, &ebo_);

    glGenBuffers(1, &vbo_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    /* Crate shader program. */
    shader_background_ = new (std::nothrow) Shader("shader_background.vert", "shader_background.frag");
    if (shader_background_ == nullptr) return false;

    shader_cad_ = new (std::nothrow) Shader("shader_model.vert", "shader_model_simple.frag");
    if (shader_cad_ == nullptr) return false;

    /* Load models. */
    for (auto map = obj2fil_map.cbegin(); map != obj2fil_map.cend(); ++map)
    {
        std::cout << log_ID_ << "Loading OpenGL "+map->first+" model." << std::endl;
        model_obj_[map->first] = new (std::nothrow) Model(map->second.c_str());
        if (model_obj_[map->first] == nullptr) return false;
    }

    /* Predefined rotation matrices. */
    root_to_ogl_ = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
                             1.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 1.0f);

    back_proj_ = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, FAR*100.f);

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921 0.0     162.202 0.0
                                 0.0     232.43  125.738 0.0
                                 0.0     0.0     1.0     0.0) */
    projection_ = glm::mat4(2.0f*(EYE_FX/FRAME_WIDTH),      0,                              0,                              0,
                            0,                              2.0f*(EYE_FY/FRAME_HEIGHT),     0,                              0,
                            2.0f*(EYE_CX/FRAME_WIDTH)-1,    2.0f*(EYE_CY/FRAME_HEIGHT)-1,   -(FAR+NEAR)/(FAR-NEAR),        -1,
                            0,                              0,                              -2.0f*(FAR*NEAR)/(FAR-NEAR),    0);

    std::cout << log_ID_ << "OpenGL renderers succesfully set up!" << std::endl;

    std::cout << log_ID_ << "Initialization completed!" << std::endl;

    return true;
}

bool SICAD::Superimpose(const ObjPoseMap & obj2pos_map, const double * cam_x, const double * cam_o, cv::Mat & img)
{
    glfwMakeContextCurrent(window_);

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
    glBindTexture(GL_TEXTURE_2D, 0);

    /* Clear the colorbuffer. */
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt())
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        shader_background_->Use();
        glUniformMatrix4fv(glGetUniformLocation(shader_background_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(back_proj_));
        glBindTexture(GL_TEXTURE_2D, texture_);
        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    if (getWireframeOpt())
    {
        /* Wireframe only. */
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    /* Use/Activate the shader. */
    shader_cad_->Use();

    /* Projection transformation matrix. */
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));

    /* View transformation matrix. */
    glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 eye_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl_) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(root_to_ogl_) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(eye_to_root) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(root_to_ogl_) * glm::mat3(eye_to_root) * glm::vec3(0.0f, -1.0f, 0.0f));

    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    /* Model transformation matrix. */
    for (auto map = obj2pos_map.cbegin(); map != obj2pos_map.cend(); ++map)
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

    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
     and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(FRAMEBUFFER_HEIGHT, FRAMEBUFFER_WIDTH, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, 0, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);
    cv::flip(ogl_pixel, ogl_pixel, 0);

    cv::resize(ogl_pixel, img, img.size(), 0, 0, cv::INTER_LINEAR);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    return true;
}


bool SICAD::getBackgroundOpt() const
{
    return show_background_;
}


void SICAD::setBackgroundOpt(bool show_background)
{
    show_background_ = show_background;;
}


bool SICAD::getWireframeOpt() const
{
    return mesh_wires_;
}


SICAD::MipMaps SICAD::getMipmapsOpt() const
{
    return mesh_mmaps_;
}
