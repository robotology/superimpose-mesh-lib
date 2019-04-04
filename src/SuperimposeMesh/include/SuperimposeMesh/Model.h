/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef MODEL_H
#define MODEL_H

#include <SuperimposeMesh/Shader.h>
#include <SuperimposeMesh/Mesh.h>

#include <vector>
#include <string>

#include <GL/glew.h>
#include <assimp/scene.h>


class Model
{
public:
    /* Functions */
    Model(const GLchar* path);

    void Draw(Shader shader);

private:
    /* Model Data */
    std::vector<Mesh> meshes;

    std::string directory;

    std::vector<Mesh::Texture> textures_loaded;

    /* Functions */
    void loadModel(std::string path);

    void processNode(aiNode* node, const aiScene* scene);

    Mesh processMesh(aiMesh* mesh, const aiScene* scene);

//    GLint TextureFromFile(const char* path, std::string directory);

//    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);
};

#endif /* MODEL_H */
