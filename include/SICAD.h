#ifndef SUPERIMPOSECAD_H
#define SUPERIMPOSECAD_H

#include "SuperImpose.h"

#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "Model.h"
#include "Shader.h"

class SICAD : public SuperImpose
{
public:
    typedef typename std::unordered_map<std::string, Model *> HandModel;

    SICAD();

    ~SICAD();

    bool Configure(GLFWwindow *& window, const ObjFileMap & obj2fil_map, const float EYE_FX, const float EYE_FY, const float EYE_CX, const float EYE_CY);

protected:
    bool Superimpose(ObjPoseMap obj2pos_map,
                     const double * cam_x, const double * cam_o,
                     cv::Mat img);

private:
    const std::string  log_ID_;
    
    GLFWwindow * window_;

    enum MipMaps
    {
        NEAREST = 0,
        LINEAR  = 1
    };

    bool      show_background_;
    bool      mesh_wires_;
    MipMaps   mesh_mmaps_;
    GLuint    texture_;
    GLuint    vao_;
    GLuint    ebo_;
    GLuint    vbo_;
    glm::mat4 root_to_ogl_;
    glm::mat4 back_proj_;
    glm::mat4 projection_;
    HandModel model_obj_;
    Shader *  shader_background_ = nullptr;
    Shader *  shader_cad_        = nullptr;

    bool    getBackgroundOpt() const;
    bool    getWireframeOpt()  const;
    MipMaps getMipmapsOpt()    const;
};

#endif /* SUPERIMPOSECAD_H */
