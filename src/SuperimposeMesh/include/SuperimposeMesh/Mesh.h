/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef MESH_H
#define MESH_H

#include <SuperimposeMesh/Shader.h>

#include <vector>

#include <assimp/scene.h>

#include <GL/glew.h>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>


class Mesh {
public:
    struct Vertex
    {
        glm::vec3 Position;
        glm::vec3 Normal;
        glm::vec2 TexCoords;
    };

    struct Texture
    {
        GLuint id;
        std::string type;
        aiString path;
    };

    Mesh(std::vector<Vertex> vertices, std::vector<GLuint> indices, std::vector<Texture> textures);

    void Draw(Shader shader);

private:
    GLuint VAO_;

    GLuint VBO_;

    GLuint EBO_;

    std::vector<Vertex> vertices_;

    std::vector<GLuint> indices_;

    std::vector<Texture> textures_;
};

#endif /* MESH_H */
