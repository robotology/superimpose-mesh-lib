#ifndef SUPERIMPOSECAD_H
#define SUPERIMPOSECAD_H

#include "Superimpose.h"

#include "Model.h"
#include "Shader.h"

#include <string>
#include <thread>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>


/**
 * A Superimpose derived class to superimpose mesh models on top of images.
 **/
class SICAD : public Superimpose
{
public:
    typedef typename std::unordered_map<std::string, std::string> ModelPathContainer;
    typedef typename std::pair<std::string, std::string>          ModelPathElement;

    typedef typename std::unordered_map<std::string, Model*>      ModelContainer;
    typedef typename std::pair<std::string, Model*>               ModelElement;

    enum class MIPMaps { nearest, linear };

    SICAD();

    SICAD(const ModelPathContainer& objfile_map,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const std::string& shader_folder,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const std::string& shader_folder,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLint num_images,
          const std::string& shader_folder,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLint num_images,
          const std::vector<float>& ogl_to_cam,
          const std::string& shader_folder,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const std::string& shader_folder,
          const bool window_visible);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const GLint num_images,
          const std::string& shader_folder,
          const bool window_visible);

    /**
     * Create a SICAD object with a dedicated OpenGL window and context.
     *
     * For the SICAD object to work properly, four shaders are needed.
     * Further, their name must be as follows:
     *
     *  - shader_model.vert for the model vertex shader
     *
     *  - shader_model.frag for the model fragment shader
     *
     *  - shader_background.vert for the background vertex shader
     *
     *  - shader_background.frag for the background fragment shader
     *
     * @param objfile_map a (tag, path) container to associate a 'tag' to the mesh file specified in 'path'.
     * @param cam_width camera or image width.
     * @param cam_height camera or image height.
     * @param cam_fx focal length along the x axis in pixels.
     * @param cam_fy focal length along the y axis in pixels.
     * @param num_images number of images to render (using glScissor) in the same window and context.
     * @param ogl_to_cam a 7 component pose vector, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, defining a camera rotation to be applied to the OpenGL camera.
     * @param shader_folder folder path containing four shaders, two for the background and two for the mesh.
     * @param window_visible true to show the rendering window, false to perform off-screen rendering.
     */
    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const GLint num_images,
          const std::vector<float>& ogl_to_cam,
          const std::string& shader_folder,
          const bool window_visible);

    bool initSICAD(const ModelPathContainer& objfile_map,
                   const GLsizei cam_width, const GLsizei cam_height,
                   const GLint num_images,
                   const std::vector<float>& ogl_to_cam,
                   const std::string& shader_folder,
                   const bool window_visible);

    bool initSICAD(const ModelPathContainer& objfile_map,
                   const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
                   const GLint num_images,
                   const std::vector<float>& ogl_to_cam,
                   const std::string& shader_folder,
                   const bool window_visible);

    virtual ~SICAD();


    bool         getOglWindowShouldClose();
    void         setOglWindowShouldClose(bool should_close);

    /**
     * Superimpose one of the mesh models loaded during SICAD object construction in a given pose from a particular camera viewpoint.
     *
     * If the size of cv::Mat img is not correct to store the result of the superimposition process, it is automatically resized.
     *
     * @note If cv::Mat img is a background image it must be of size cam_width*cam_height, as specified during object construction, and the SICAD::setBackgroundOpt(bool show_background)
     * must have been evoked with true.
     *
     * @param objpos_map a (tag, pose) container to associate a 7 component 'pose', (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation
     * @param img an image where the result of the superimposition is stored
     *
     * @return true upon success, false otherswise.
     **/
    bool         superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img) override;

    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img);

    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool         setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool         getBackgroundOpt() const;
    void         setBackgroundOpt(bool show_background);

    GLenum       getWireframeOpt()  const;
    void         setWireframeOpt(bool show_mesh_wires);

    MIPMaps      getMipmapsOpt()    const;

    int          getTilesNumber()   const;
    int          getTilesRows()     const;
    int          getTilesCols()     const;

private:
    bool initOGL(const GLsizei width, const GLsizei height, const GLint num_viewports, const bool window_visibile);

    bool is_initialized_  = false;
    bool has_proj_matrix_ = false;


    static int         class_counter_;
    static GLsizei     renderbuffer_size_;


    const std::string  log_ID_             = "[SI-CAD]";

    GLFWwindow       * window_             = nullptr;
    GLint              tiles_num_          = 0;
    GLsizei            tiles_cols_         = 0;
    GLsizei            tiles_rows_         = 0;
    GLsizei            image_width_        = 0;
    GLsizei            image_height_       = 0;
    GLsizei            window_width_       = 0;
    GLsizei            window_height_      = 0;
    glm::mat3          ogl_to_cam_         = glm::mat3(1.0f);
    GLsizei            framebuffer_width_  = 0;
    GLsizei            framebuffer_height_ = 0;
    GLsizei            render_img_width_   = 0;
    GLsizei            render_img_height_  = 0;
    const GLfloat      near_               = 0.001f;
    const GLfloat      far_                = 1000.0f;

    std::thread::id    main_thread_id_;

    bool               show_background_    = false;
    GLenum             show_mesh_mode_     = GL_FILL;
    MIPMaps            mesh_mmaps_         = MIPMaps::nearest;
    Shader           * shader_background_  = nullptr;
    Shader           * shader_cad_         = nullptr;
    ModelContainer     model_obj_;
    GLuint             fbo_;
    GLuint             texture_color_buffer_;
    GLuint             texture_depth_buffer_;
    GLuint             texture_background_;
    GLuint             vao_;
    GLuint             ebo_;
    GLuint             vbo_;
    glm::mat4          back_proj_;
    glm::mat4          projection_;

    glm::mat4          getViewTransformationMatrix(const double* cam_x, const double* cam_o);


    void               pollOrPostEvent();


    void               setBackground(cv::Mat& img);
    void               setWireframe(GLenum mode);
    static void        callbackKeypress(GLFWwindow* window, int key, int scancode, int action, int mode);


    void               factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height);
};

#endif /* SUPERIMPOSECAD_H */
