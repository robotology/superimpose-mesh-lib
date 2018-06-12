# 📜 SuperimposeMesh changelog


## Version 0.9.104.0
##### `CMake`
 - Devel branch will now have +100 on the patch number to differentiate from master branch.
 - Use CONFIG mode for Assimp, fix inclusion of headers and linking to Assimp.
##### `Dependencies`
 - Minimum required version of Assimp is now `3.3.0`.

## Version 0.9.4.0
##### `Bugfix`
 - Fix tests, some shader files were not copied in the binary test folder.
 - Fix exported dependencies, GLFW3 and GLEW are now lower case.


## Version 0.9.3.0
##### `Changed behavior`
 - The library now renders on a dedicated framebuffer and is no longer capable of showing the rendering status onscreen.
   - Future version may enable this behavior in some way or completely remove the creation of a rendering window at all.

##### `Feature`
 - It is now possible to draw/render a reference frame instead of a mesh by passing "frame" as object name to ModelPoseContainer.

##### `Bugfix`
 - Windows platform can now render any image size (limited to discrete card capabilities).
 - Fixed an error in the camera projection matrix that was causing small position drift at increasing distances.


## Version 0.9.2.0
### Updates
##### `Bugfix`
 - Windows is now fully supported and tested.

##### `Test`
 - Add Appveyor and Travis yml files. Tests still need to be improved.


## Version 0.9.1.0
### Updates
##### `CMake`
 - Update installation helper files, which are updated from YCM commit f162fcb.


## Version 0.9.0.0

 - This is the first public release of the SuperimposeMesh library.
