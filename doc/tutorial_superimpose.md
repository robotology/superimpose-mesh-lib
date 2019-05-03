# %Superimpose an object {#tutorial_superimpose}

This tutorial introduces the basic steps to superimpose an object, i.e. a mesh,
on top of an image by using the [SICAD](@ref SICAD) class and its methods.<br>

Just have a look at the following commented, ready-to-go, code snippets!<br>

## Default easy-peasy SICAD

---

[View it online.](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/tutorial_superimpose.cpp "tutorial_superimpose.cpp")
\includelineno tutorial_code/tutorial_superimpose.cpp

If instead you want to provide your own OpenGL shader files the code changes
just a tiny bit: you just have to provide two more paramenters to the the
[SICAD](@ref SICAD) constructor.

## Use your own OpenGL shaders in SICAD

---

[View it online.](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/tutorial_superimpose_customshader.cpp "tutorial_superimpose_customshader.cpp")
\includelineno tutorial_code/tutorial_superimpose_customshader.cpp

Here are the shaders and the Space Invaders mesh model:
 - [Model vertex shader](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/shader_model.vert "Model vertex shader")
 - [Textured model vertex shader](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/shader_model_texture.vert "Textured model vertex shader")
 - [Model fragment shader](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/shader_model.frag "Model fragment shader")
 - [Background vertex shader](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/shader_background.vert "Background vertex shader")
 - [Background fragment shader](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/shader_background.frag "Background fragment shader")
 - [Space Invader fiend](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/spaceinvader.obj "Space Invader fiend")

⚠️ The four shaders must have the following **exact names**:
 - _shader_model.vert_ for the model vertex shader
 - _shader_model_texture.vert_ for the textured model vertex shader
 - _shader_model.frag_ for the model fragment shader
 - _shader_background.vert_ for the background vertex shader
 - _shader_background.frag_ for the background fragment shader

## Result + Next

---

You should get something like this:
![👾](alien.jpg)

You can now proceed to the next tutorial: [how to add a background](@ref tutorial_superimpose_background)!
