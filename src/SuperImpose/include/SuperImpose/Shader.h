#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>


class Shader
{
public:
  	/* The program ID. */
	GLuint Program;
    
	/* Constructor reads and builds the shader. */
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath);
  	
    /* Use the program. */
  	void Use();
};

#endif /* SHADER_H */
