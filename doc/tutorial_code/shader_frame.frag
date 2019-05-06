/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#version 330 core

in vec3 vert_color;

out vec4 frag_color;

void main()
{
    frag_color = vec4(vert_color, 1.0f);
}
