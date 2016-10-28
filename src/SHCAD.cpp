#include "SHCAD.h"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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


SHCAD::SHCAD(GLFWwindow * window, const ObjFileMap & obj2fil_map, const float EYE_L_FX, const float EYE_L_FY, const float EYE_L_CX, const float EYE_L_CY) :
        _log_ID("[SH-CAD]"), _window(window), _obj2fil_map(obj2fil_map), _EYE_L_FX(EYE_L_FX), _EYE_L_FY(EYE_L_FY), _EYE_L_CX(EYE_L_CX), _EYE_L_CY(EYE_L_CY) {

    std::cout << _log_ID << "Setting up OpenGL renderers.";
    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(_window);

    /* Create a background texture. */
    glGenTextures(1, &_texture);

    /* Crate the squared support for the backround texture. */
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);
    GLfloat vertices[] = {// Positions    // Colors           // Texture Coords
                           1.0f,  1.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,      // Top Right
                           1.0f, -1.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,      // Bottom Right
                          -1.0f, -1.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,      // Bottom Left
                          -1.0f,  1.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f       // Top Left
    };

    GLuint indices[] = { 0, 1, 3,   // First Triangle
                         1, 2, 3 }; // Second Triangle

    /* Create and bind an element buffer object. */
    glGenBuffers(1, &_ebo);

    glGenBuffers(1, &_vbo);

    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    /* Crate shader program. */
    _shader_background = new Shader("shader_background.vert", "shader_background.frag");
    _shader_cad        = new Shader("shader_model.vert"     , "shader_model_simple.frag");

    /* Load models. */
    for (auto map = _obj2fil_map.cbegin(); map != _obj2fil_map.cend(); ++map)
    {
        std::cout << _log_ID << "Loading OpenGL "+map->first+" model.";
        _model_obj[map->first] = new Model(map->second.c_str());
    }

    /* Predefined rotation matrices. */
    _root_to_ogl = glm::mat4(0.0f, 0.0f, 1.0f, 0.0f,
                             1.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 1.0f);

    _back_proj = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, FAR*100.f);

    /* Projection matrix. */
    /* Intrinsic camera matrix: (232.921 0.0     162.202 0.0
     0.0     232.43  125.738 0.0
     0.0     0.0     1.0     0.0) */
    _projection = glm::mat4(2.0f*EYE_L_FX/FRAME_WIDTH,       0,                                  0,                           0,
                            0,                               2.0f*EYE_L_FY/FRAME_HEIGHT,         0,                           0,
                            2.0f*(EYE_L_CX/FRAME_WIDTH)-1,   2.0f*(EYE_L_CY/FRAME_HEIGHT)-1,     -(FAR+NEAR)/(FAR-NEAR),     -1,
                            0,                               0,                                  -2.0f*FAR*NEAR/(FAR-NEAR),   0 );

    std::cout << _log_ID << "OpenGL renderers succesfully set up!";

    std::cout << _log_ID << "Initialization completed!";
}


bool SHCAD::superimposeHand(ObjPoseMap obj2pos_map,
                            const double * cam_x, const double * cam_o,
                            cv::Mat img)
{
    unsigned char * ogl_pixel = new unsigned char [3 * FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];

    /* Load and generate the texture. */
    glBindTexture(GL_TEXTURE_2D, _texture);

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

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    /* Clear the colorbuffer. */
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt())
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        _shader_background->Use();
        glUniformMatrix4fv(glGetUniformLocation(_shader_background->Program, "projection"), 1, GL_FALSE, glm::value_ptr(_back_proj));
        glBindTexture(GL_TEXTURE_2D, _texture);
        glBindVertexArray(_vao);
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
    _shader_cad->Use();

    /* View transformation matrix. */
    /* Extrinsic camera matrix: */
    glm::mat4 root_eye_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 root_eye_o = glm::rotate(glm::mat4(1.0f), static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::mat3(_root_to_ogl) * glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z),
                                 glm::mat3(_root_to_ogl) * (glm::vec3(root_eye_t[3].x, root_eye_t[3].y, root_eye_t[3].z) + glm::mat3(root_eye_o) * glm::vec3(0.0f, 0.0f, 1.0f)),
                                 glm::mat3(_root_to_ogl) * glm::mat3(root_eye_o) * glm::vec3(0.0f, -1.0f, 0.0f));

    glUniformMatrix4fv(glGetUniformLocation(_shader_cad->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    glUniformMatrix4fv(glGetUniformLocation(_shader_cad->Program, "projection"), 1, GL_FALSE, glm::value_ptr(_projection));

    /* Model transformation matrix. */
    for (auto map = obj2pos_map.cbegin(); map != obj2pos_map.cend(); ++map)
    {
        double * j_x = obj2pos_map[map->first].first;
        double * j_o = obj2pos_map[map->first].second;

        glm::mat4 root_j_t = glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(j_x[0]), static_cast<float>(j_x[1]), static_cast<float>(j_x[2])));
        glm::mat4 root_j_o = glm::rotate(glm::mat4(1.0f), static_cast<float>(j_o[3]), glm::vec3(static_cast<float>(j_o[0]), static_cast<float>(j_o[1]), static_cast<float>(j_o[2])));

        glm::mat4 model = _root_to_ogl * (root_j_t * root_j_o);

        glUniformMatrix4fv(glGetUniformLocation(_shader_cad->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

        _model_obj[map->first]->Draw(*_shader_cad);
    }

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, ogl_pixel);
    for (size_t i = 0; i < (FRAMEBUFFER_HEIGHT / 2); ++i) {
        unsigned char(&row_bot)[3 * FRAMEBUFFER_WIDTH] = *reinterpret_cast<unsigned char(*)[3 * FRAMEBUFFER_WIDTH]>(&ogl_pixel[3 * FRAMEBUFFER_WIDTH * i]);
        unsigned char(&row_up) [3 * FRAMEBUFFER_WIDTH] = *reinterpret_cast<unsigned char(*)[3 * FRAMEBUFFER_WIDTH]>(&ogl_pixel[3 * FRAMEBUFFER_WIDTH * (FRAMEBUFFER_HEIGHT-1 - i)]);
        std::swap(row_bot, row_up);
    }

    /* Swap the buffers. */
    glfwSwapBuffers(_window);

    delete [] ogl_pixel;

    return true;
}


SHCAD::~SHCAD() {
    std::cout << _log_ID << "Deallocating OpenGL resources.";

    for (auto map = _model_obj.begin(); map != _model_obj.end(); ++map)
    {
        std::cout << _log_ID << "Deleting OpenGL "+map->first+" model.";
        delete map->second;
    }
    glDeleteVertexArrays(1, &_vao);
    glDeleteBuffers     (1, &_ebo);
    glDeleteBuffers     (1, &_vbo);
    glDeleteTextures    (1, &_texture);

    std::cout << _log_ID << "Deleting OpenGL shaders.";
    delete _shader_background;
    delete _shader_cad;

    std::cout << _log_ID << "Closing OpenGL context.";
    glfwMakeContextCurrent(NULL);

    std::cout << _log_ID << "Closing OpenGL window.";
    glfwSetWindowShouldClose(_window, GL_TRUE);

    std::cout << _log_ID << "OpenGL resource deallocation completed!";
}


bool SHCAD::getBackgroundOpt()
{
    return _show_background;
}


bool SHCAD::getWireframeOpt()
{
    return _mesh_wires;
}


SHCAD::MipMaps SHCAD::getMipmapsOpt()
{
    return _mesh_mmaps;
}
