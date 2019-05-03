# %Superimpose an object with the background {#tutorial_superimpose_background}

This tutorial is based upon the [previous one](@ref tutorial_superimpose) and
explain how to add a background image to the rendering process.
Our good ol' fiend 👾 from Space Invaders will finally have _Space_ behind
him.

![👾's Space](space.png)

One of the main reason to why this is important, and to why actually this
library exists, is that in computer vision and augmented-reality applications
you want your mesh to be in a specific pose relative to a camera viewpoint.
As a consequence, you usually have an image grabbed from a camera and the pose
of an object relative to it.

Here is another commented, ready-to-go, code snippet (using default shaders)!<br>

[View it online.](https://github.com/robotology/superimpose-mesh-lib/blob/master/doc/tutorial_code/tutorial_background.cpp "tutorial_background.cpp")
\includelineno tutorial_code/tutorial_background.cpp

You should get something like this:
![👾 in the Space](alien_space.jpg)
