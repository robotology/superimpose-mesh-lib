#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>

#include <GL/glew.h>
#include <assimp/scene.h>

#include "Shader.h"
#include "Mesh.h"


class Model
{
public:
    /* Functions. */
    Model(const GLchar* path);
    void Draw(Shader shader);
    
private:
    /* Model Data. */
    std::vector<Mesh> meshes;
    std::string directory;
    std::vector<Texture> textures_loaded;
    
    /* Functions. */
    void loadModel(std::string path);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh, const aiScene* scene);
//    GLint TextureFromFile(const char* path, std::string directory);
//    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);
};

#endif /* MODEL_H */
