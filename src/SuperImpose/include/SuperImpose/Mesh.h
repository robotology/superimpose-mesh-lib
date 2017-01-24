#ifndef MESH_H
#define MESH_H

#include <vector>

#include <GL/glew.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <assimp/scene.h>

#include "Shader.h"


struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};


struct Texture {
    GLuint id;
    std::string type;
    aiString path;
};
/* Note: structs have a great property in C++, their memory layout is sequential. */


class Mesh {
public:
    /* Mesh Data. */
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Texture> textures;
    
    /* Functions. */
    Mesh(std::vector<Vertex> vertices, std::vector<GLuint> indices, std::vector<Texture> textures);
    void Draw(Shader shader);
    
private:
    /* Render data. */
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;
    
    /* Functions. */
    void setupMesh();
};

#endif /* MESH_H */
