#ifndef SHADER_H
#define SHADER_H

#include <exception>

#include <GL/glew.h>


class Shader
{
public:
	/* Constructor reads and builds the shader. */
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath);
  	
    /* Use the program. */
  	void Use();

    /* The program ID. */
    GLuint Program;
};

#endif /* SHADER_H */
