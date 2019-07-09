/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#version 330 core

layout (location = 0) out vec4 color;
layout (location = 1) out float depth;

float near = 0.001f;
float far = 1000.0f;

float linearize_depth(float coord_z)
{
    /* Back to NDC. */
    float z = coord_z * 2.0 - 1.0;

    /* Evaluate real depth. */
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    /* BGR orange-like color. */
    color = vec4(0.2f, 0.5f, 1.0f, 1.0f);

    /* Real depth. */
    depth = linearize_depth(gl_FragCoord.z);
}
