#ifndef SHADER_H
#define SHADER_H

#include <exception>

#include <GL/glew.h>


class Shader
{
public:
	/* Constructor reads and builds the shader. */
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath);

    /* Install (i.e. use) the program object. */
    void install();

    /* Uninstall (i.e. use) the program object. */
    void uninstall();

    /* The program ID. */
    GLuint Program;
};

#endif /* SHADER_H */
