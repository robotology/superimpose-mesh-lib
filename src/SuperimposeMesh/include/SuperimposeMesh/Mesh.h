/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef MESH_H
#define MESH_H

#include "Shader.h"

#include <vector>

#include <GL/glew.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <assimp/scene.h>


struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};


struct Texture {
    GLuint      id;
    std::string type;
    aiString    path;
};


class Mesh {
public:
    /* Functions */
    Mesh(std::vector<Vertex> vertices, std::vector<GLuint> indices, std::vector<Texture> textures);

    void Draw(Shader shader);

    /* Mesh Data */
    std::vector<Vertex>  vertices;
    std::vector<GLuint>  indices;
    std::vector<Texture> textures;

private:
    /* Render data */
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;

    /* Functions */
    void setupMesh();
};

#endif /* MESH_H */
