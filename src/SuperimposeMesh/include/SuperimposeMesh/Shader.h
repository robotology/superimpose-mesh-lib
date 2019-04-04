/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

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
