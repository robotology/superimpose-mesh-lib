#include "SuperimposeMesh/Mesh.h"

#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


Mesh::Mesh
(
    std::vector<Vertex> vertices,
    std::vector<GLuint> indices,
    std::vector<Texture> textures
)
{
    this->vertices = vertices;
    this->indices  = indices;
    this->textures = textures;

    this->setupMesh();
}


void Mesh::setupMesh()
{
    glGenVertexArrays(1, &this->VAO);
    glGenBuffers(1, &this->VBO);
    glGenBuffers(1, &this->EBO);

    glBindVertexArray(this->VAO);

    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(GLuint), &this->indices[0], GL_STATIC_DRAW);

    /* Vertex Positions */
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    /* Vertex Normals. */
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)sizeof(glm::vec3));
    glEnableVertexAttribArray(1);

    /* Vertex Texture Coords */
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(glm::vec3)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}


void Mesh::Draw(Shader shader)
{
//    GLuint diffuseNr  = 1;
//    GLuint specularNr = 1;
//    for (GLuint i = 0; i < this->textures.size(); ++i)
//    {
//        /* Activate proper texture unit before binding. */
//        glActiveTexture(GL_TEXTURE0 + i);
//
//        /* Retrieve texture number (the N in diffuse_textureN). */
//        std::string number;
//        std::string name = this->textures[i].type;
//        /* Transfer GLuint to stream. */
//        if(name == "texture_diffuse")
//        {
//            number = std::to_string(diffuseNr++);
//        }
//        else if(name == "texture_specular")
//        {
//            number = std::to_string(specularNr++);
//        }
//
//        glUniform1i(glGetUniformLocation(shader.Program, (name + number).c_str()), i);
//        glBindTexture(GL_TEXTURE_2D, this->textures[i].id);
//    }
//    glActiveTexture(GL_TEXTURE0);

    /* Draw mesh. */
    glBindVertexArray(this->VAO);
    glDrawElements(GL_TRIANGLES, this->indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
