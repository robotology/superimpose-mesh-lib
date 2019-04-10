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
	/**
     * Constructor reads and builds the shader.
     */
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath);

    /**
     * Activate the shader program.
     */
    void install();

    /**
     * Deactivate the shader program.
     */
    void uninstall();


    inline const GLuint get_program()
    {
        return shader_program_id_;
    }

private:
    /**
     * The program ID.
     */
    GLuint shader_program_id_;
};

#endif /* SHADER_H */
