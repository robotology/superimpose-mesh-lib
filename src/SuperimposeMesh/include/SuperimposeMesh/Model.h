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

#include <assimp/scene.h>

#include <GL/glew.h>


class Model
{
public:
    Model(const GLchar* path);

    void Draw(Shader shader);

    bool has_texture();

protected:
    void loadModel(std::string path);

    void processNode(aiNode* node, const aiScene* scene);

    Mesh processMesh(aiMesh* mesh, const aiScene* scene);

    GLint TextureFromFile(const char* path, std::string directory);

    std::vector<Mesh::Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);

private:
    std::vector<Mesh> meshes_;

    std::string directory_;

    std::vector<Mesh::Texture> textures_loaded_;
};

#endif /* MODEL_H */
