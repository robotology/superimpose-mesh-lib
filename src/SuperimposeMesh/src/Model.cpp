/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include "SuperimposeMesh/Model.h"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <glm/glm.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


Model::Model(const GLchar* path)
{
    loadModel(path);
}


void Model::Draw(Shader shader)
{
    for(GLuint i = 0; i < meshes_.size(); i++)
    {
        meshes_[i].Draw(shader);
    }
}


bool Model::has_texture()
{
    return (textures_loaded_.size() > 0 ? true : false);
}


void Model::loadModel(std::string path)
{
    Assimp::Importer import;
    const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cerr << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
        return;
    }

    size_t foundpos = path.find_last_of('/');
    if (foundpos == std::string::npos)
    {
       directory_ = ".";
    }
    else
    {
       directory_ = path.substr(0, foundpos);
    }

    processNode(scene->mRootNode, scene);
}


void Model::processNode(aiNode* node, const aiScene* scene)
{
    /* Process all the node's meshes (if any). */
    for (GLuint i = 0; i < node->mNumMeshes; ++i)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes_.push_back(processMesh(mesh, scene));
    }

    /* Then do the same for each of its children. */
    for (GLuint i = 0; i < node->mNumChildren; ++i)
    {
        processNode(node->mChildren[i], scene);
    }
}


Mesh Model::processMesh(aiMesh* mesh, const aiScene* scene)
{
    std::vector<Mesh::Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Mesh::Texture> textures;

    /* Process vertices. */
    for (GLuint i = 0; i < mesh->mNumVertices; ++i)
    {
        Mesh::Vertex vertex;

        /* Process vertex positions, normals and texture coordinates. */
        vertex.Position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        vertex.Normal = glm::vec3(mesh->mNormals[i].x,  mesh->mNormals[i].y,  mesh->mNormals[i].z);

        /* Does the mesh contain texture coordinates? */
        if (mesh->mTextureCoords[0])
        {
            vertex.TexCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        }
        else
        {
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);
        }

        vertices.push_back(vertex);
    }

    /* Process indices. */
    for (GLuint i = 0; i < mesh->mNumFaces; ++i)
    {
        aiFace face = mesh->mFaces[i];
        for (GLuint j = 0; j < face.mNumIndices; ++j)
        {
            indices.push_back(face.mIndices[j]);
        }
    }

    /* Process textures. */
    /* The texture code is taken as-is. The tutorial on texture was skipped. */
    if (mesh->mMaterialIndex > 0)
    {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

        std::vector<Mesh::Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

        std::vector<Mesh::Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    }

    return Mesh(vertices, indices, textures);
}


std::vector<Mesh::Texture> Model::loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName)
{
    std::vector<Mesh::Texture> textures;
    for (GLuint i = 0; i < mat->GetTextureCount(type); ++i)
    {
        aiString str;
        mat->GetTexture(type, i, &str);
        GLboolean skip = false;
        for (GLuint j = 0; j < textures_loaded_.size(); ++j)
        {
            if (textures_loaded_[j].path == str)
            {
                textures.push_back(textures_loaded_[j]);
                skip = true;
                break;
            }
        }

        /* If texture hasn't been loaded already, load it. */
        if (!skip)
        {
            Mesh::Texture texture;

            texture.id = TextureFromFile(str.C_Str(), directory_);
            texture.type = typeName;
            texture.path = str;
            textures.push_back(texture);

            /* Add to loaded textures. */
            textures_loaded_.push_back(texture);
        }
    }

    return textures;
}


GLint Model::TextureFromFile(const char* path, std::string directory)
{
    std::string filename = directory + "/" + std::string(path);
    cv::Mat image = cv::imread(filename, cv::IMREAD_ANYCOLOR);

    /* Generate texture ID and load texture data. */
    GLuint textureID;
    glGenTextures(1, &textureID);

    /* Assign texture to ID. */
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.ptr());
    glGenerateMipmap(GL_TEXTURE_2D);

    /* Parameters. */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    return textureID;
}
