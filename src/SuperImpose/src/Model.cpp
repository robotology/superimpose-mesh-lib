#include "SuperImpose/Model.h"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
// #include <SOIL/SOIL.h>


Model::Model(const GLchar* path) {
    this->loadModel(path);
}


void Model::Draw(Shader shader) {
    for(GLuint i = 0; i < this->meshes.size(); i++) {
        this->meshes[i].Draw(shader);
    }
}


void Model::loadModel(std::string path) {
    Assimp::Importer import;
    const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);
    
    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
        return;
    }
    
    size_t foundpos = path.find_last_of('/');
    if (foundpos == std::string::npos) {
        this->directory = ".";
    } else {
        this->directory = path.substr(0, foundpos);
    }
    
    this->processNode(scene->mRootNode, scene);
}


void Model::processNode(aiNode* node, const aiScene* scene) {
    /* Process all the node's meshes (if any). */
    for (GLuint i = 0; i < node->mNumMeshes; ++i) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        this->meshes.push_back(this->processMesh(mesh, scene));
    }
    
    /* Then do the same for each of its children. */
    for (GLuint i = 0; i < node->mNumChildren; ++i) {
        this->processNode(node->mChildren[i], scene);
    }
}
/* Note: we could basically forget about processing any of the nodes and simply loop through all of the scene's meshes directly without doing all this complicated stuff with indices. The reason we're doing this is that the initial idea for using nodes like this is that it defines a parent-child relation between meshes. By recursively iterating through these relations we can actually define certain meshes to be parents of other meshes. It is generally recommended to stick with this approach for whenever you want extra control over your mesh data. These node-like relations are after all defined by the artists who created the models. */


Mesh Model::processMesh(aiMesh* mesh, const aiScene* scene) {
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Texture> textures;
    
    /* Process vertices. */
    for (GLuint i = 0; i < mesh->mNumVertices; ++i) {
        Vertex vertex;
        
        /* Process vertex positions, normals and texture coordinates. */
        vertex.Position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        
        vertex.Normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        
        /* Does the mesh contain texture coordinates? */
        if (mesh->mTextureCoords[0]) {
            vertex.TexCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        } else {
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);
        }
        /* Note: Assimp allows a model to have up to 8 different texture coordinates per vertex, i.e. mTextureCoords[0:7][*], which we're not going to use. We only care about the first set of texture coordinates. */
        
        vertices.push_back(vertex);
    }
    
    /* Process indices. */
    /* Assimp's interface defines meshes with an array of faces, where each face represents a single primitive which, in our case (due to the aiProcess_Triangulate option), are always triangles. A face contains the indices that define which vertices we need to draw in what order for each primitive so if we iterate over all the faces and store all the face's indices in the indices vector we're all set. */
    for (GLuint i = 0; i < mesh->mNumFaces; ++i) {
        aiFace face = mesh->mFaces[i];
        for (GLuint j = 0; j < face.mNumIndices; ++j) {
            indices.push_back(face.mIndices[j]);
        }
    }
    
//    /* Process textures. */
//    /* The texture code is taken as-is. The tutorial on texture was skipped. */
//    if (mesh->mMaterialIndex > 0) {
//        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
//        
//        std::vector<Texture> diffuseMaps = this->loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
//        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
//        
//        std::vector<Texture> specularMaps = this->loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
//        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
//    }

    return Mesh(vertices, indices, textures);
}


//std::vector<Texture> Model::loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName) {
//    std::vector<Texture> textures;
//    for (GLuint i = 0; i < mat->GetTextureCount(type); ++i) {
//        aiString str;
//        mat->GetTexture(type, i, &str);
//        GLboolean skip = false;
//        for (GLuint j = 0; j < textures_loaded.size(); ++j) {
//            if (textures_loaded[j].path == str) {
//                textures.push_back(textures_loaded[j]);
//                skip = true;
//                break;
//            }
//        }
//        /* If texture hasn't been loaded already, load it. */
//        if (!skip) {
//            Texture texture;
//            texture.id = TextureFromFile(str.C_Str(), this->directory);
//            texture.type = typeName;
//            texture.path = str;
//            textures.push_back(texture);
//            
//            /* Add to loaded textures. */
//            this->textures_loaded.push_back(texture);
//        }
//    }
//    
//    return textures;
//}


//GLint Model::TextureFromFile(const char* path, std::string directory)
//{
//    /* Generate texture ID and load texture data. */
//    std::string filename = std::string(path);
//    filename = directory + "/" + filename;
//    GLuint textureID;
//    glGenTextures(1, &textureID);
//    int width;
//    int height;
//    unsigned char* image = SOIL_load_image(filename.c_str(), &width, &height, 0, SOIL_LOAD_RGB);
//    
//    /* Assign texture to ID. */
//    glBindTexture(GL_TEXTURE_2D, textureID);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
//    glGenerateMipmap(GL_TEXTURE_2D);
//    
//    /* Parameters. */
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    glBindTexture(GL_TEXTURE_2D, 0);
//    SOIL_free_image_data(image);
//    
//    return textureID;
//}
