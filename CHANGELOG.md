# 📜 SuperimposeMesh changelog

## 🔖 Version 0.10.0
##### `Changed behavior`
 - SICAD constructs now always require the intrinsic camera parameters.
 - SICAD object construction cannot be procrastinated to a later call of SICAD::initSICAD().
 - SICAD objects now always perform off-screen rendering and the constructor's window_visible parameter has been removed.

##### `CMake`
 - The CMake target now has namespace SI:: instead of SuperimposeMesh::.
 - Devel branch will now have +100 on the patch number to differentiate from master branch.
 - assimp library is now serched using YCM Findassimp.cmake.
 - glew library is now serched using YCM FindGLEW.cmake.
 - Use CONFIG mode for glfw3.
 - AddInstallRPATHSupport.cmake is now used from YCM.
 - InstallBasicPackageFiles.cmake is now used from YCM.
 - Import AddDependenciesPrintUtils from robotology/YARP - robotology/visual-tracking-control to visualize the status of dependencies.
 - Removed explicit dependency to OpenGL.
 - Tests are now correctly generated for Visual Studio generator.

##### `Feature`
 - SICAD class now provide default OpenGL shaders. The behaviour is the following:
   - By not providing a shader folder path to SICAD constructor, the default shaders will be used.
   - By providing a shader folder path to SICAD constructor, the default shaders will be bypassed.
   Please read the updated documentation of the SICAD class constructors to have a full understanding of this new capability.
 - Add SICAD::superimpose() methods to render images to Pixel Buffer Objects (PBO).
 - Add SICAD::getPBOs() and SICDAD::getPBO(size_t) to access the allow the user to read the PBOs and manipulate the memory on the GPU.
    As an example, the user may map the PBO on a CUDA/OpenCL-enabled GPU and do some processing on the images directly on the GPU, without wasting CPU time and copy on memory overheads.
 - Add SICAD::setMipmapsOpt(SICAD::MIPMaps) public method to change the type of MIPMaps. Can be MIPMaps::nearest or MIPMaps::linear.
 - Improved tests.
 - SICAD will now automatically find mesh textures and select the proper shader program to render them.

##### `Dependencies`
 - Added mandatory dependency from YCM release `0.10.2` and above.
 - Minimum required version of Assimp is now `3.3.0`.
 - Minimum required version of OpenCV is now `2.4.9`.

##### `Test`
 - Fixed both AppVeyor and Travis tests.
 - Added tests for mesh with textures.
 - Added tests to use custom, user-provided, OpenGL shaders.


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
