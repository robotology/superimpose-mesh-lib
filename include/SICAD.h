#ifndef SUPERIMPOSEHANDCAD_H
#define SUPERIMPOSEHANDCAD_H

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

    SICAD (GLFWwindow * window, const ObjFileMap & obj2fil_map, const float EYE_L_FX, const float EYE_L_FY, const float EYE_L_CX, const float EYE_L_CY);

    ~SICAD();

protected:
    bool superimpose(ObjPoseMap obj2pos_map,
                     const double * cam_x, const double * cam_o,
                     cv::Mat img);

private:
    const std::string  _log_ID;
    GLFWwindow *       _window;
    const ObjFileMap & _obj2fil_map;
    const float        _EYE_L_FX;
    const float        _EYE_L_FY;
    const float        _EYE_L_CX;
    const float        _EYE_L_CY;

    enum MipMaps
    {
        NEAREST = 0,
        LINEAR  = 1
    };

    bool      _show_background;
    bool      _mesh_wires;
    MipMaps   _mesh_mmaps;
    GLuint    _texture;
    GLuint    _vao;
    GLuint    _ebo;
    GLuint    _vbo;
    glm::mat4 _root_to_ogl;
    glm::mat4 _back_proj;
    glm::mat4 _projection;
    HandModel _model_obj;
    Shader *  _shader_background = nullptr;
    Shader *  _shader_cad        = nullptr;

    bool    getBackgroundOpt() const;
    bool    getWireframeOpt()  const;
    MipMaps getMipmapsOpt()    const;
};

#endif /* SUPERIMPOSEHANDCAD_H */
