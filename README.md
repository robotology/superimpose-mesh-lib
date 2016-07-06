# superimpose-hand

It is possible to view the iCub skeleton of the right hand superimposed on a yarpview GUI with the left eye.

It is possible to dump data for
 1. end effector pose
    * YARP port: /superimpose_hand/endeffector_pose:o
 2. left camera pose
     * YARP port: /superimpose_hand/left_camera_pose:o
 3. left camera images
     * YARP port: /superimpose_hand/img_skeleton_left:o

 ### Dependencies
 - [ICUB](https://wiki.icub.org/wiki/Main_Page)
     - Debian, MacOS, WIN: [icub-main on GitHub](https://github.com/robotology/icub-main)
 - [Yet Another Robot Platform, YARP](http://www.yarp.it)
     - Debian, MacOS, WIN: [yarp on GitHub](https://github.com/robotology/yarp)
 - [OpenCV](http://opencv.org)
     - Debian, MacOS, WIN: [OpenCV on GitHub](https://github.com/opencv/opencv)
 - [GLFW](http://www.glfw.org)
     - Debian: apt
     - MacOS: brew
     - WIN: TBT
 - [OpenGL Mathematics, GLM](http://glm.g-truc.net)
     - Debian: apt
     - MacOS: brew
     - WIN: TBD
 - [OpenGL Extension Wrangler, GLEW](http://glew.sourceforge.net)
     - Debian: apt
     - MacOS: brew
     - WIN: TBD
 - [Open Asset Import Library, ASSIMP](http://assimp.org)
     - Debian: apt
     - MacOS: brew
     - WIN: TBD
