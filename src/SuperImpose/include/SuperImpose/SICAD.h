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
public:
    typedef typename std::unordered_map<std::string, Model*> ObjModel;

    enum MipMaps
    {
        NEAREST = 0,
        LINEAR  = 1
    };

    SICAD(const ObjFileMap& objfile_map);

    SICAD(const ObjFileMap& objfile_map, const int cam_width, const int cam_height, const float eye_fx, const float eye_fy, const float eye_cx, const float eye_cy);

    virtual ~SICAD();


    static bool initOGL(const GLsizei width, const GLsizei height, const GLint viewports = 1);

    static int  oglWindowShouldClose();

    bool        superimpose(const ObjPoseMap& objpos_map,
                            const double* cam_x, const double* cam_o,
                            cv::Mat& img);

    bool        superimpose(const ObjPoseMap& objpos_map,
                            const double* cam_x, const double* cam_o,
                            const int cam_width, const int cam_height, const float eye_fx, const float eye_fy, const float eye_cx, const float eye_cy,
                            cv::Mat& img);

    bool        superimpose(const std::vector<ObjPoseMap>& objpos_multimap,
                            const double* cam_x, const double* cam_o,
                            cv::Mat& img);
    
    bool        superimpose(const std::vector<ObjPoseMap>& objpos_multimap,
                            const double* cam_x, const double* cam_o,
                            const int cam_width, const int cam_height, const float eye_fx, const float eye_fy, const float eye_cx, const float eye_cy,
                            cv::Mat& img);

    bool        getBackgroundOpt() const;
    void        setBackgroundOpt(bool show_background);

    GLenum      getWireframeOpt()  const;
    void        setWireframeOpt(bool show_mesh_wires);

    MipMaps     getMipmapsOpt()    const;

private:
    static bool                     can_init;
    static GLFWwindow             * window_;
    static GLint                    viewports_;
    static GLsizei                  window_width_;
    static GLsizei                  window_height_;
    static GLsizei                  framebuffer_width_;
    static GLsizei                  framebuffer_height_;
    constexpr static const GLfloat  near_ = 0.001f;
    constexpr static const GLfloat  far_  = 1000.0f;

    const std::string  log_ID_;
    bool               show_background_   = false;
    GLenum             show_mesh_mode_    = GL_FILL;
    MipMaps            mesh_mmaps_        = NEAREST;
    Shader           * shader_background_ = nullptr;
    Shader           * shader_cad_        = nullptr;
    ObjModel           model_obj_;
    GLuint             texture_;
    GLuint             vao_;
    GLuint             ebo_;
    GLuint             vbo_;
    glm::mat4          root_to_ogl_;
    glm::mat4          back_proj_;
    glm::mat4          projection_;

    void        set_background(cv::Mat& img, const unsigned int unit = 0);
    void        set_wireframe(GLenum mode);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
};

#endif /* SUPERIMPOSECAD_H */
