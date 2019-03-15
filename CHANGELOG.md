# 📜 SuperimposeMesh changelog

## 🔖 Version 0.10.100
##### `Changed behavior`
 - SICAD constructs now always require the intrinsic camera parameters.
 - SICAD object construction cannot be procrastinated to a later call of SICAD::initSICAD().
 - SICAD objects now always perform off-screen rendering and the constructor's window_visible parameter has been removed.

##### `CMake`
 - Devel branch will now have +100 on the patch number to differentiate from master branch.
 - Use CONFIG mode for Assimp, fix inclusion of headers and linking to Assimp.
 - Use CONFIG mode for glfw3.
 - Update AddInstallRPATHSupport.cmake from robotology/YCM v0.9.1.
 - Update InstallBasicPackageFiles.cmake from robotology/YCM v0.9.1.
 - Import AddDependenciesPrintUtils from robotology/YARP - robotology/visual-tracking-control to visualize the status of dependencies.
 - Removed explicit dependency to OpenGL.

##### `Feature`
  - Add SICAD::superimpose() methods to render images to Pixel Buffer Objects (PBO).
  - Add SICAD::getPBOs() and SICDAD::getPBO(size_t) to access the allow the user to read the PBOs and manipulate the memory on the GPU.
    As an example, the user may map the PBO on a CUDA/OpenCL-enabled GPU and do some processing on the images directly on the GPU, without wasting CPU time and copy on memory overheads.
  - Add SICAD::setMipmapsOpt(SICAD::MIPMaps) public method to change the type of MIPMaps. Can be MIPMaps::nearest or MIPMaps::linear.
  - Improved tests.

##### `Dependencies`
 - Minimum required version of Assimp is now `3.3.0`.


## 🔖 Version 0.9.4.0
##### `Bugfix`
 - Fix tests, some shader files were not copied in the binary test folder.
 - Fix exported dependencies, GLFW3 and GLEW are now lower case.


## 🔖 Version 0.9.3.0
##### `Changed behavior`
 - The library now renders on a dedicated framebuffer and is no longer capable of showing the rendering status onscreen.
   - Future version may enable this behavior in some way or completely remove the creation of a rendering window at all.

##### `Feature`
 - It is now possible to draw/render a reference frame instead of a mesh by passing "frame" as object name to ModelPoseContainer.

##### `Bugfix`
 - Windows platform can now render any image size (limited to discrete card capabilities).
 - Fixed an error in the camera projection matrix that was causing small position drift at increasing distances.


## 🔖 Version 0.9.2.0
##### `Bugfix`
 - Windows is now fully supported and tested.

##### `Test`
 - Add Appveyor and Travis yml files. Tests still need to be improved.


## 🔖 Version 0.9.1.0
##### `CMake`
 - Update installation helper files, which are updated from YCM commit f162fcb.


## 🔖 Version 0.9.0.0

 - This is the first public release of the SuperimposeMesh library.
