#ifndef SUPERIMPOSECAD_H
#define SUPERIMPOSECAD_H

#include "Superimpose.h"

#include "Model.h"
#include "Shader.h"

#include <utility>
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

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const GLint num_images);

    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const GLint num_images,
          const std::string& shader_folder);

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
     * @param num_images number of images (i.e. viewports) rendered in the same GL context.
     * @param ogl_to_cam a 7-component pose vector, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, defining a camera rotation applied to the OpenGL camera.
     * @param shader_folder folder path containing four shaders, two for the background and two for the mesh.
     * @param window_visible true to show the rendering window, false to perform off-screen rendering.
     */
    SICAD(const ModelPathContainer& objfile_map,
          const GLsizei cam_width, const GLsizei cam_height,
          const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy,
          const GLint num_images,
          const std::string& shader_folder,
          const std::vector<float>& ogl_to_cam);

    virtual ~SICAD();


    bool getOglWindowShouldClose();
    void setOglWindowShouldClose(bool should_close);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then creates an image of the mesh models as they are seen by the virtual camera.
     *
     * @note If cv::Mat `img` is a background image it must be of size `cam_width * cam_height`, as specified during object construction,
     * and the `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param img An image representing the result of the superimposition. The variable is automatically resized if its size is not correct to store the entire result of the superimposition.
     *
     * @return true upon success, false otherswise.
     **/
    bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img) override;

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then creates a single image tiling the viewports in a regular grid.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @note If cv::Mat `img` is a background image it must be of size `cam_width * cam_height`, as specified during object construction,
     * and the `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param img An image representing the result of the superimposition. The variable is automatically resized if its size is not correct to store the entire result of the superimposition.
     *
     * @return true upon success, false otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img);

    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then stores the pixels of the mesh models as they are seen by the virtual camera in the `pbo_index`-th Pixel Buffer Object (PBO).
     *
     * @note By invoking this command rendered pixels are stored in the `pbo_index`-th PBO and, in order to use it, the OpenGL context must remain current.
     * As a consequence, once you are done working with the `pbo_index`-th PBO (can be accessed by means of `SICAD::getPBO(pbo_index)`) and before invoking again
     * any other `SICAD::superimpose()` function, you must invoke `SICAD::releaseContext()`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, const size_t pbo_index);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then stores the pixels of the mesh models as they are seen by the virtual camera in the `pbo_index`-th Pixel Buffer Object (PBO).
     *
     * @note `img` must be of size `cam_width * cam_height`, as specified during object construction, and the
     * `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     * @param img A background image.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, const size_t pbo_index, const cv::Mat& img);

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then stores the pixels of the viewports in the `pbo_index`-th Pixel Buffer Object (PBO) by tiling them in a regular grid.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, const size_t pbo_index);

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then stores the pixels of the viewports in the `pbo_index`-th Pixel Buffer Object (PBO) by tiling them in a regular grid.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @note `img` must be of size `cam_width * cam_height`, as specified during object construction, and the
     * `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     * @param img A background image.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, const size_t pbo_index, const cv::Mat& img);

    /**
     * Make the current thread OpenGL context not current.
     *
     * 
     */
    virtual void releaseContext() const;

    /**
     * Returns the Pixel Buffer Object (PBO) vector and its size.
     *
     * @note SICAD class retains the ownership of the PBO.
     *
     * @return (PBO base array address, number of PBOs)
     */
    std::pair<const GLuint*, size_t> getPBOs() const;

    /**
     * Returns `pbo_index`-th Pixel Buffer Object (PBO).
     *
     * @note SICAD class retains the ownership of the PBO.
     *
     * @return (true, PBO) if `pbo_index` exists, (false, 0) otherwise.
     */
    std::pair<bool, GLuint> getPBO(const size_t pbo_index) const;

    bool setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool getBackgroundOpt() const;

    void setBackgroundOpt(bool show_background);

    GLenum getWireframeOpt() const;

    void setWireframeOpt(bool show_mesh_wires);

    MIPMaps getMipmapsOpt() const;

    int getTilesNumber() const;

    int getTilesRows() const;

    int getTilesCols() const;

private:
    bool has_proj_matrix_ = false;


    static int class_counter_;
    static GLsizei renderbuffer_size_;


    const std::string log_ID_             = "[SI-CAD]";

    GLFWwindow*   window_             = nullptr;
    GLint         tiles_num_          = 0;
    GLsizei       tiles_cols_         = 0;
    GLsizei       tiles_rows_         = 0;
    GLsizei       image_width_        = 0;
    GLsizei       image_height_       = 0;
    glm::mat3     ogl_to_cam_         = glm::mat3(1.0f);
    GLsizei       framebuffer_width_  = 0;
    GLsizei       framebuffer_height_ = 0;
    GLsizei       tile_img_width_     = 0;
    GLsizei       tile_img_height_    = 0;
    const GLfloat near_               = 0.001f;
    const GLfloat far_                = 1000.0f;

    std::thread::id main_thread_id_;

    bool           show_background_    = false;
    GLenum         show_mesh_mode_     = GL_FILL;
    MIPMaps        mesh_mmaps_         = MIPMaps::nearest;
    Shader*        shader_background_  = nullptr;
    Shader*        shader_cad_         = nullptr;
    Shader*        shader_frame_       = nullptr;
    ModelContainer model_obj_;
    GLuint         fbo_;
    GLuint         texture_color_buffer_;
    GLuint         texture_depth_buffer_;
    GLuint         texture_background_;
    GLuint         vao_background_;
    GLuint         ebo_background_;
    GLuint         vbo_background_;
    GLuint         vao_frame_;
    GLuint         vbo_frame_;
    size_t         pbo_number_ = 2;
    GLuint         pbo_[2];
    glm::mat4      back_proj_;
    glm::mat4      projection_;

    glm::mat4 getViewTransformationMatrix(const double* cam_x, const double* cam_o);


    void pollOrPostEvent();

    void renderBackground(const cv::Mat& img) const;
    
    void setWireframe(GLenum mode);

    void factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height);
};

#endif /* SUPERIMPOSECAD_H */
