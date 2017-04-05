#ifndef SUPERIMPOSECAD_H
#define SUPERIMPOSECAD_H

#include "SuperImpose.h"

#include <string>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "Model.h"
#include "Shader.h"


class SICAD : public SuperImpose
{
protected:
    typedef typename std::unordered_map<std::string, Model*> ObjModel;

    enum MIPMaps
    {
        NEAREST = 0,
        LINEAR  = 1
    };

public:
    SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, std::string shader_folder);

    SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLint image_num, std::string shader_folder);

    SICAD(const ObjFileMap& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLint image_num, std::string shader_folder,
          const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    virtual ~SICAD();


    bool        initOGL(const GLsizei width, const GLsizei height, const GLint num_viewports = 1);

    bool        getOglWindowShouldClose();
    void        setOglWindowShouldClose(bool should_close);

    bool        superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img);

    bool        superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img);

    bool        superimpose(const ObjPoseMap& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                            const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool        superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                            const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool        setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool        getBackgroundOpt() const;
    void        setBackgroundOpt(bool show_background);

    GLenum      getWireframeOpt()  const;
    void        setWireframeOpt(bool show_mesh_wires);

    MIPMaps     getMipmapsOpt()    const;

    int         getTilesNumber()   const;
    int         getTilesRows()     const;
    int         getTilesCols()     const;

private:
    const std::string  log_ID_             = "[SI-CAD]";

    GLFWwindow       * window_             = nullptr;
    GLint              tiles_num_          = 0;
    GLsizei            tiles_cols_         = 0;
    GLsizei            tiles_rows_         = 0;
    GLsizei            image_width_        = 0;
    GLsizei            image_height_       = 0;
    GLsizei            window_width_       = 0;
    GLsizei            window_height_      = 0;
    GLsizei            framebuffer_width_  = 0;
    GLsizei            framebuffer_height_ = 0;
    GLsizei            render_img_width_   = 0;
    GLsizei            render_img_height_  = 0;
    const GLfloat      near_               = 0.001f;
    const GLfloat      far_                = 1000.0f;

    bool               show_background_    = false;
    GLenum             show_mesh_mode_     = GL_FILL;
    MIPMaps            mesh_mmaps_         = NEAREST;
    Shader           * shader_background_  = nullptr;
    Shader           * shader_cad_         = nullptr;
    ObjModel           model_obj_;
    GLuint             texture_;
    GLuint             vao_;
    GLuint             ebo_;
    GLuint             vbo_;
    glm::mat4          root_to_ogl_;
    glm::mat4          back_proj_;
    glm::mat4          projection_;

    void               set_background(cv::Mat& img);
    void               set_wireframe(GLenum mode);
    static void        key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);

    void               factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height);
};

#endif /* SUPERIMPOSECAD_H */
