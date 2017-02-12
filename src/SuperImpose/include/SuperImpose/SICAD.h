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

    SICAD(GLFWwindow*& window, const ObjFileMap& objfile_map, const float EYE_FX, const float EYE_FY, const float EYE_CX, const float EYE_CY);

    virtual ~SICAD();


    bool        superimpose(const ObjPoseMap& objpos_map,                   const double* cam_x, const double* cam_o, cv::Mat& img);
    bool        superimpose(const std::vector<ObjPoseMap>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img);

    bool        getBackgroundOpt() const;
    void        setBackgroundOpt(bool show_background);

    GLenum      getWireframeOpt()  const;
    void        setWireframeOpt(bool show_mesh_wires);

    MipMaps     getMipmapsOpt()    const;

private:
    const std::string  log_ID_;
    
    GLFWwindow       * window_;

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
};

#endif /* SUPERIMPOSECAD_H */
