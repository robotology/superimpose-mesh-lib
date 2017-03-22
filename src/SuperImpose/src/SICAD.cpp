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


SICAD::SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLint num_images, std::string shader_folder)
{
    if (!initOGL(cam_width, cam_height, num_images))
        throw std::runtime_error("ERROR::SICAD::CTOR::OPENGL\nERROR: Could not initialize OpenGL.");

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


    /* Crate background shader program. */
    std::cout << log_ID_ << "Setting up background shader." << std::endl;

    try
    {
        shader_background_ = new (std::nothrow) Shader((shader_folder + "/shader_background.vert").c_str(), (shader_folder + "/shader_background.frag").c_str());
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error(e.what());
    }
    if (shader_background_ == nullptr)
        throw std::runtime_error("ERROR::SICAD::CTOR::SHADER\nERROR: Background shader source file not found!");

    std::cout << log_ID_ << "Background shader succesfully set up!" << std::endl;


    /* Crate model shader program. */
    std::cout << log_ID_ << "Setting up model shader." << std::endl;

    try
    {
        shader_cad_ = new (std::nothrow) Shader((shader_folder + "/shader_model.vert").c_str(), (shader_folder + "/shader_model.frag").c_str());
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error(e.what());
    }
    if (shader_cad_ == nullptr)
        throw std::runtime_error("ERROR::SICAD::CTOR::SHADER\nERROR: 3D model shader source file not found!");

    std::cout << log_ID_ << "Model shader succesfully set up!" << std::endl;


    /* Load models. */
    for (auto map = objfile_map.cbegin(); map != objfile_map.cend(); ++map)
    {
        std::cout << log_ID_ << "Loading OpenGL " + map->first + " model." << std::endl;
        model_obj_[map->first] = new (std::nothrow) Model(map->second.c_str());
        if (model_obj_[map->first] == nullptr)
            throw std::runtime_error("ERROR::SICAD::CTOR::OBJ\nERROR: File " + map->second + " not found!");
    }


    /* Fixed rotation matrices from root to OpenGL frame. */
    root_to_ogl_ = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
                             1.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 1.0f);

    back_proj_ = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, far_*100.f);

    std::cout << log_ID_ << "OpenGL renderers succesfully set up!" << std::endl;

    std::cout << log_ID_ << "Initialization completed!" << std::endl;
}


SICAD::SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, std::string shader_folder) :
    SICAD(objfile_map, cam_width, cam_height, 1, shader_folder) { }


SICAD::SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLint num_images, std::string shader_folder,
             const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy) :
    SICAD(objfile_map, cam_width, cam_height, num_images, shader_folder)
{
    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);
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


bool SICAD::initOGL(const GLsizei width, const GLsizei height, const GLint num_images)
{
    std::cout << log_ID_ << "Start setting up..." << std::endl;


    /* Initialize GLFW. */
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << log_ID_ << "Failed to initialize GLFW.";
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


    /* Create test window to enquire for OpenGL for the maximum size of the renderbuffer */
    window_ = glfwCreateWindow(1, 1, "OpenGL renderbuffer test", nullptr, nullptr);
    glfwMakeContextCurrent(window_);


    /* Enquire GPU for maximum size (both width and height) of the framebuffer */
    GLsizei rb_size;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &rb_size);
    std::cout << log_ID_ << "Max renderbuffer size is "+std::to_string(rb_size)+"x"+std::to_string(rb_size)+" size." << std::endl;


    /* Close the test window */
    glfwDestroyWindow(window_);


    /* Given image size */
    image_width_  = width;
    image_height_ = height;
    std::cout << log_ID_ << "Given image size "+std::to_string(image_width_)+"x"+std::to_string(image_height_)+"." << std::endl;


    /* Compute the maximum number of images that can be rendered conditioned on the maximum framebuffer size */
    factorize_int(num_images, std::floor(rb_size / image_width_), std::floor(rb_size / image_height_), tiles_cols_, tiles_rows_);
    tiles_num_ = tiles_rows_ * tiles_cols_;
    std::cout << log_ID_ << "Required to render "+std::to_string(num_images)+" image(s)." << std::endl;
    std::cout << log_ID_ << "Allowed number or rendered images is "+std::to_string(tiles_num_)+" ("+std::to_string(tiles_rows_)+"x"+std::to_string(tiles_cols_)+" grid)." << std::endl;


    /* Create a window. */
    window_ = glfwCreateWindow(image_width_ * tiles_cols_, image_height_ * tiles_rows_, "OpenGL Window", nullptr, nullptr);
    if (window_ == nullptr)
    {
        std::cerr << log_ID_ << "Failed to create GLFW window.";
        glfwTerminate();
        return false;
    }
    glfwGetWindowSize(window_, &window_width_, &window_height_);
    std::cout << log_ID_ << "Window created with size "+std::to_string(window_width_)+"x"+std::to_string(window_height_)+"." << std::endl;


    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);


    /* Set window callback functions. */
    glfwSetKeyCallback(window_, key_callback);


    /* Initialize GLEW to use the OpenGL implementation provided by the videocard manufacturer. */
    /* Note: remember that the OpenGL are only specifications, the implementation is provided by the manufacturers. */
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cerr << log_ID_ << "Failed to initialize GLEW.";
        return false;
    }


    /* Set default OpenGL viewport for the current window. */
    /* Note that framebuffer_width_ and framebuffer_height_ may differ w.r.t. width and height in hdpi monitors. */
    glfwGetFramebufferSize(window_, &framebuffer_width_, &framebuffer_height_);
    glViewport(0, 0, framebuffer_width_, framebuffer_height_);
    std::cout << log_ID_ << "The window framebuffer size is "+std::to_string(framebuffer_width_)+"x"+std::to_string(framebuffer_height_)+"." << std::endl;


    /* Set rendered image size. May vary in HDPI monitors. */
    render_img_width_  = framebuffer_width_  / tiles_cols_;
    render_img_height_ = framebuffer_height_ / tiles_rows_;
    std::cout << log_ID_ << "The rendered image size is "+std::to_string(render_img_width_)+"x"+std::to_string(render_img_height_)+"." << std::endl;


    /* Set GL property. */
    glEnable(GL_DEPTH_TEST);

    glfwPollEvents();

    std::cout << log_ID_ << "Succesfully set up!" << std::endl;

    return true;
}


int SICAD::oglWindowShouldClose()
{
    return glfwWindowShouldClose(window_);
}


bool SICAD::superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    glfwMakeContextCurrent(window_);

    /* Render in the upper-left-most tile of the render grid */
    glViewport(0,                 framebuffer_height_ - render_img_height_,
               render_img_width_, render_img_height_                       );
    glScissor (0,                 framebuffer_height_ - render_img_height_,
               render_img_width_, render_img_height_                       );

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt()) set_background(img);

    /* View mesh filled or as wireframe. */
    set_wireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 root_eye_t  = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 eye_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl_) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(root_to_ogl_) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(eye_to_root) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(root_to_ogl_) * glm::mat3(eye_to_root) * glm::vec3(0.0f, -1.0f, 0.0f));

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
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
    shader_cad_->uninstall();

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_ / tiles_rows_, framebuffer_width_ / tiles_cols_, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, framebuffer_height_ - render_img_height_, render_img_width_, render_img_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, ogl_pixel, 0);
    cv::resize(ogl_pixel, img, cv::Size(image_width_, image_height_), 0, 0, cv::INTER_LINEAR);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    return true;
}


bool SICAD::superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    /* View transformation matrix. */
    glm::mat4 root_eye_t  = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 eye_to_root = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(root_to_ogl_) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(root_to_ogl_) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(eye_to_root) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(root_to_ogl_) * glm::mat3(eye_to_root) * glm::vec3(0.0f, -1.0f, 0.0f));

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    for (unsigned int i = 0; i < tiles_rows_; ++i)
    {
        for (unsigned int j = 0; j < tiles_cols_; ++j)
        {
            /* Multimap index */
            int idx = i * tiles_cols_ + j;

            /* Render starting by the upper-left-most tile of the render grid, proceding by columns and rows. */
            glViewport(render_img_width_ * j, framebuffer_height_ - (render_img_height_ * (i + 1)),
                       render_img_width_    , render_img_height_                                   );
            glScissor (render_img_width_ * j, framebuffer_height_ - (render_img_height_ * (i + 1)),
                       render_img_width_    , render_img_height_                                   );

            /* Clear the colorbuffer. */
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            /* Draw the background picture. */
            if (getBackgroundOpt()) set_background(img);

            /* View mesh filled or as wireframe. */
            set_wireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            shader_cad_->install();
            for (auto map = objpos_multimap[idx].cbegin(); map != objpos_multimap[idx].cend(); ++map)
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
            shader_cad_->uninstall();
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


bool SICAD::superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                        const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

    superimpose(objpos_map, cam_x, cam_o, img);
    
    return true;
}


bool SICAD::superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                        const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

    superimpose(objpos_multimap, cam_x, cam_o, img);

    return true;
}


bool SICAD::setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    glfwMakeContextCurrent(window_);

    /* Projection matrix. */
    /* See: https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL */
    /* Intrinsic camera matrix: (232.921      0.0     162.202    0.0
                                   0.0      232.43    125.738    0.0
                                   0.0        0.0       1.0      0.0) */
    projection_ = glm::mat4(2.0f*(cam_fx/cam_width),    0,                              0,                                  0,
                            0,                          2.0f*(cam_fy/cam_height),       0,                                  0,
                            1-2.0f*(cam_cx/cam_width),  1-2.0f*(cam_cy/cam_height),    -(far_+near_)/(far_-near_),         -1,
                            0,                          0,                             -2.0f*(far_*near_)/(far_-near_),     0);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    shader_cad_->uninstall();

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


SICAD::MIPMaps SICAD::getMipmapsOpt() const
{
    return mesh_mmaps_;
}


int SICAD::getTilesNumber() const
{
    return tiles_rows_ * tiles_cols_;
}


int SICAD::getTilesRows() const
{
    return tiles_rows_;
}


int SICAD::getTilesCols() const
{
    return tiles_cols_;
}


void SICAD::set_background(cv::Mat& img)
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

    /* Install/Use the program specified by the shader. */
    shader_background_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_background_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(back_proj_));
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    shader_background_->uninstall();
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


void SICAD::factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height)
{
    double sqrt_area = std::sqrt(static_cast<double>(area));
    height = std::min(static_cast<int>(std::ceil(sqrt_area)),     height_limit);
    width  = std::min(static_cast<int>(std::ceil(area / height)), width_limit);
}
