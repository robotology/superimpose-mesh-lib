#include "SuperimposeMesh/SICAD.h"

#include <iostream>
#include <exception>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>


int     SICAD::class_counter_     = 0;
GLsizei SICAD::renderbuffer_size_ = 0;


SICAD::SICAD() {}


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const bool window_visible) :
    SICAD(objfile_map,
          320, 240,
          1,
          {1.0, 0.0, 0.0, 0.0},
          ".",
          window_visible) { }


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          320, 240,
          1,
          {1.0, 0.0, 0.0, 0.0},
          shader_folder,
          window_visible) { }


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          cam_width, cam_height,
          1,
          {1.0, 0.0, 0.0, 0.0},
          shader_folder,
          window_visible) { }


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height,
             const GLint num_images,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          cam_width, cam_height,
          num_images,
          {1.0, 0.0, 0.0, 0.0},
          shader_folder,
          window_visible) { }


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height,
             const GLint num_images,
             const std::vector<float>& ogl_to_cam,
             const std::string& shader_folder,
             const bool window_visible)
{
    initSICAD(objfile_map,
              cam_width, cam_height,
              num_images,
              ogl_to_cam,
              shader_folder,
              window_visible);
}


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          cam_width, cam_height,
          1,
          {1.0, 0.0, 0.0, 0.0},
          shader_folder,
          window_visible)
{
    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);
}


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
             const GLint num_images,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          cam_width, cam_height,
          num_images,
          {1.0, 0.0, 0.0, 0.0},
          shader_folder,
          window_visible)
{
    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);
}


SICAD::SICAD(const ModelPathContainer& objfile_map,
             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
             const GLint num_images,
             const std::vector<float>& ogl_to_cam,
             const std::string& shader_folder,
             const bool window_visible) :
    SICAD(objfile_map,
          cam_width, cam_height,
          num_images,
          ogl_to_cam,
          shader_folder,
          window_visible)
{
    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);
}



SICAD::~SICAD()
{
    std::cout << log_ID_ << "Deallocating OpenGL resources..." << std::endl;

    glfwMakeContextCurrent(window_);

    for (const ModelElement& pair : model_obj_)
    {
        std::cout << log_ID_ << "Deleting OpenGL "+ pair.first+" model." << std::endl;
        delete pair.second;
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

    class_counter_--;
    if (class_counter_ == 0)
    {
        std::cout << log_ID_ << "Terminating GLFW." << std::endl;
        glfwTerminate();
    }

    std::cout << log_ID_ << "OpenGL resource deallocation completed!" << std::endl;
}


bool SICAD::initSICAD(const ModelPathContainer &objfile_map,
                      const GLsizei cam_width, const GLsizei cam_height,
                      const GLint num_images,
                      const std::vector<float> &ogl_to_cam,
                      const std::string &shader_folder,
                      const bool window_visible)
{
    if (is_initialized_)
    {
        std::cout << "INFO::SICAD::InitSICAD\nINFO: already initialized." << std::endl;
        return false;
    }

    if (ogl_to_cam.size() != 4)
        throw std::runtime_error("ERROR::SICAD::CTOR::OGL_TO_CAM\nERROR: Wrong size provided. Should be 4, was given " + std::to_string(ogl_to_cam.size()) + ".");

    if (!initOGL(cam_width, cam_height, num_images, window_visible))
        throw std::runtime_error("ERROR::SICAD::CTOR::OPENGL\nERROR: Could not initialize OpenGL.");

    std::cout << log_ID_ << "Setting up OpenGL renderers." << std::endl;


    /* Rotation from real camera to OpenGL frame */
    ogl_to_cam_ = glm::mat3(glm::rotate(glm::mat4(1.0f), ogl_to_cam[3], glm::make_vec3(ogl_to_cam.data())));

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
    for (const ModelPathElement& pair : objfile_map)
    {
        std::cout << log_ID_ << "Loading OpenGL " + pair.first + " model." << std::endl;
        model_obj_[pair.first] = new (std::nothrow) Model(pair.second.c_str());
        if (model_obj_[pair.first] == nullptr)
            throw std::runtime_error("ERROR::SICAD::CTOR::OBJ\nERROR: File " + pair.second + " not found!");
    }

    back_proj_ = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, far_*100.f);

    std::cout << log_ID_ << "OpenGL renderers succesfully set up!" << std::endl;


    /* Increase static class counter. */
    ++class_counter_;

    /* Allow the class to be used. */
    is_initialized_ = true;

    std::cout << log_ID_ << "Initialization completed!" << std::endl;

    return true;
}


bool SICAD::initSICAD(const ModelPathContainer& objfile_map,
                      const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
                      const GLint num_images,
                      const std::vector<float>& ogl_to_cam,
                      const std::string& shader_folder,
                      const bool window_visible)
{
    if (!initSICAD(objfile_map,
                   cam_width, cam_height,
                   num_images,
                   ogl_to_cam,
                   shader_folder,
                   window_visible))
        return false;

    std::cout << log_ID_ << "Setting up default projection matrix." << std::endl;

    if (setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy))
        return false;

    return true;
}


bool SICAD::initOGL(const GLsizei width, const GLsizei height, const GLint num_images, const bool window_visibile)
{
    std::cout << log_ID_ << "Start setting up..." << std::endl;


    /* Initialize GLFW. */
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << log_ID_ << "Failed to initialize GLFW.";
        return false;
    }


    /* Set context properties by "hinting" specific (property, value) pairs. */
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,    3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,    3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,           GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE,                GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE,                  window_visibile);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,    GL_TRUE);
#ifdef GLFW_MAC
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);
#endif


    if (renderbuffer_size_ == 0)
    {
        /* Create test window to enquire for OpenGL for the maximum size of the renderbuffer */
        window_ = glfwCreateWindow(1, 1, "OpenGL renderbuffer test", nullptr, nullptr);
        glfwMakeContextCurrent(window_);

        /* Enquire GPU for maximum renderbuffer size (both width and height) of the default framebuffer */
        glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &renderbuffer_size_);
        std::cout << log_ID_ << "Max renderbuffer size is " + std::to_string(renderbuffer_size_) + "x"+std::to_string(renderbuffer_size_) + " size." << std::endl;

        /* Close the test window */
        glfwDestroyWindow(window_);
    }


    /* Given image size */
    image_width_  = width;
    image_height_ = height;
    std::cout << log_ID_ << "Given image size " + std::to_string(image_width_) + "x" + std::to_string(image_height_) + "." << std::endl;


    /* Compute the maximum number of images that can be rendered conditioned on the maximum renderbuffer size */
    factorize_int(num_images, std::floor(renderbuffer_size_ / image_width_), std::floor(renderbuffer_size_ / image_height_), tiles_cols_, tiles_rows_);
    tiles_num_ = tiles_rows_ * tiles_cols_;
    std::cout << log_ID_ << "Required to render " + std::to_string(num_images) + " image(s)." << std::endl;
    std::cout << log_ID_ << "Allowed number or rendered images is " + std::to_string(tiles_num_) + " (" + std::to_string(tiles_rows_) + "x" + std::to_string(tiles_cols_) + " grid)." << std::endl;


    /* Create a window. */
    window_ = glfwCreateWindow(image_width_ * tiles_cols_, image_height_ * tiles_rows_, "OpenGL Window", nullptr, nullptr);
    if (window_ == nullptr)
    {
        std::cerr << log_ID_ << "Failed to create GLFW window.";
        glfwTerminate();
        return false;
    }
    glfwGetWindowSize(window_, &window_width_, &window_height_);
    std::cout << log_ID_ << "Window created with size " + std::to_string(window_width_) + "x" + std::to_string(window_height_) + "." << std::endl;


    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);


    /* Set window callback functions. */
    glfwSetKeyCallback(window_, callbackKeypress);


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
    std::cout << log_ID_ << "The window framebuffer size is " + std::to_string(framebuffer_width_) + "x" + std::to_string(framebuffer_height_) + "." << std::endl;


    /* Set rendered image size. May vary in HDPI monitors. */
    render_img_width_  = framebuffer_width_  / tiles_cols_;
    render_img_height_ = framebuffer_height_ / tiles_rows_;
    std::cout << log_ID_ << "The rendered image size is " + std::to_string(render_img_width_) + "x" + std::to_string(render_img_height_) + "." << std::endl;


    /* Set GL property. */
    glEnable(GL_DEPTH_TEST);

    glfwPollEvents();
    main_thread_id_ = std::this_thread::get_id();

    std::cout << log_ID_ << "Succesfully set up!" << std::endl;

    return true;
}


bool SICAD::getOglWindowShouldClose()
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::getOglWindowShouldClose\nERROR: not initialized." << std::endl;
        return false;
    }

    return (glfwWindowShouldClose(window_) == GL_TRUE ? true : false);
}


void SICAD::setOglWindowShouldClose(bool should_close)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::setOglWindowShouldClose\nERROR: not initialized." << std::endl;
        return;
    }

    glfwSetWindowShouldClose(window_, GL_TRUE);

    pollOrPostEvent();
}


bool SICAD::superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: not initialized." << std::endl;
        return false;
    }

    if (!has_proj_matrix_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: projection matrix not set." << std::endl;
        return false;
    }

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
    if (getBackgroundOpt()) setBackground(img);

    /* View mesh filled or as wireframe. */
    setWireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    /* Model transformation matrix. */
    for (const ModelPoseContainerElement& pair : objpos_map)
    {
        const double* pose = pair.second.data();

        glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        model[3][0] = pose[0];
        model[3][1] = pose[1];
        model[3][2] = pose[2];

        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

        model_obj_[pair.first]->Draw(*shader_cad_);
    }
    shader_cad_->uninstall();

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_ / tiles_rows_, framebuffer_width_ / tiles_cols_, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, framebuffer_height_ - render_img_height_, render_img_width_, render_img_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, img, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    return true;
}


bool SICAD::superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: not initialized." << std::endl;
        return false;
    }

    if (!has_proj_matrix_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: projection matrix not set." << std::endl;
        return false;
    }


    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

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
            if (getBackgroundOpt()) setBackground(img);

            /* View mesh filled or as wireframe. */
            setWireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            shader_cad_->install();
            for (const ModelPoseContainerElement& pair : objpos_multimap[idx])
            {
                const double* pose = pair.second.data();

                glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
                model[3][0] = static_cast<float>(pose[0]);
                model[3][1] = static_cast<float>(pose[1]);
                model[3][2] = static_cast<float>(pose[2]);

                glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

                model_obj_[pair.first]->Draw(*shader_cad_);
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

    cv::flip(ogl_pixel, img, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    return true;
}


bool SICAD::superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                        const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: not initialized." << std::endl;
        return false;
    }

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

    superimpose(objpos_map, cam_x, cam_o, img);
    
    return true;
}


bool SICAD::superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                        const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::superimpose\nERROR: not initialized." << std::endl;
        return false;
    }

    setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);

    superimpose(objpos_multimap, cam_x, cam_o, img);

    return true;
}


bool SICAD::setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy)
{
    if (!is_initialized_)
    {
        std::cerr << "ERROR::SICAD::setProjectionMatrix\nERROR: not initialized." << std::endl;
        return false;
    }

    glfwMakeContextCurrent(window_);

    /* Projection matrix. */
    /* In both OpenGL window coordinates and HZ image coordinate systems, (0,0) is the lower left corner with X and Y increasing right and up, respectively.
       In a normal image file, the (0,0) pixel is in the upper left corner. We have code paths to deal with this in one of two ways:
       first, we can draw our images upside down, so that all the pixel-based coordinate systems are the same. This is the code path used when “window_coords=’y up’”.
       Second, we can keep the images right side up and modify the projection matrix so that OpenGL will generate window coordinates that compensate for the flipped image coordinates.
       In this “window_coords=’y down’” path, the generated OpenGL Y window coordinates are (height-y).
       
       Enough of the preliminaries. We calculate the OpenGL Projection matrix when window_coords==’y up’ to be:
       [2*K00/width,  -2*K01/width,   (width - 2*K02 + 2*x0)/width,                            0]
       [          0, -2*K11/height, (height - 2*K12 + 2*y0)/height,                            0]
       [          0,             0, (-zfar - znear)/(zfar - znear), -2*zfar*znear/(zfar - znear)]
       [          0,             0,                             -1,                            0]
       
       With window_coords==’y down’, we have:
       [2*K00/width, -2*K01/width,    (width - 2*K02 + 2*x0)/width,                            0]
       [          0, 2*K11/height, (-height + 2*K12 + 2*y0)/height,                            0]
       [          0,            0,  (-zfar - znear)/(zfar - znear), -2*zfar*znear/(zfar - znear)]
       [          0,            0,                              -1,                            0]
       
       Where Knm is the (n,m) entry of the 3x3 HZ instrinsic camera calibration matrix K. (K is upper triangular and scaled such that the lower-right entry is one.)
       Width and height are the size of the camera image, in pixels, and x0 and y0 are the camera image origin and are normally zero.
       Znear and zfar are the standard OpenGL near and far clipping planes, respectively. */
    projection_ = glm::mat4(2.0f*(cam_fx/cam_width),    0,                              0,                                  0,
                            0,                          2.0f*(cam_fy/cam_height),       0,                                  0,
                            1-2.0f*(cam_cx/cam_width),  1-2.0f*(cam_cy/cam_height),    -(far_+near_)/(far_-near_),         -1,
                            0,                          0,                             -2.0f*(far_*near_)/(far_-near_),     0);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    shader_cad_->uninstall();

    has_proj_matrix_ = true;

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


glm::mat4 SICAD::getViewTransformationMatrix( const double* cam_x, const double* cam_o)
{
    glm::mat4 root_cam_t  = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 cam_to_root = glm::rotate(glm::mat4(1.0f),
                                        static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::vec3(root_cam_t[3].x, root_cam_t[3].y, root_cam_t[3].z),
                                 glm::vec3(root_cam_t[3].x, root_cam_t[3].y, root_cam_t[3].z) + glm::mat3(cam_to_root) * ogl_to_cam_ * glm::vec3(0.0f, 0.0f, -1.0f),
                                 glm::mat3(cam_to_root) * ogl_to_cam_ * glm::vec3(0.0f, 1.0f, 0.0f));

    return view;
}


void SICAD::pollOrPostEvent()
{
    if(main_thread_id_ == std::this_thread::get_id())
        glfwPollEvents();
    else
        glfwPostEmptyEvent();
}


void SICAD::setBackground(cv::Mat& img)
{
    /* Load and generate the texture. */
    glBindTexture(GL_TEXTURE_2D, texture_);

    /* Set the texture wrapping/filtering options (on the currently bound texture object). */
    if (getMipmapsOpt() == MIPMaps::nearest)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    else if (getMipmapsOpt() == MIPMaps::linear)
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


void SICAD::setWireframe(GLenum mode)
{
    glPolygonMode(GL_FRONT_AND_BACK, mode);
}


void SICAD::callbackKeypress(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    /* When a user presses the escape key, we set the WindowShouldClose property to true, closing the application. */
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}


void SICAD::factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height)
{
    double sqrt_area = std::floor(std::sqrt(static_cast<double>(area)));
    height = std::min(static_cast<int>(sqrt_area), height_limit);
    width  = std::min(area / height,               width_limit);
}
