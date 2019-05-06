#include <SuperimposeMesh/SICAD.h>

#include <cmath>
#include <exception>
#include <iostream>

#include <glm/gtc/matrix_transform.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main()
{
    /**
     * We want to rendere a single mesh on 1 OpenGL viewport.
     * The SICAD class can be profitably used to accomplish this!
     *
     * We first need some parameters to properly draw the object.
     * These parameters are the hypothetical intrinsic camera parameters that
     * would be used, or that is actually used, to take grab the image on which
     * we want to superimpose the mesh.
     *
     * For example, we suppose to have a 320x240 camera with focal length
     * of 257.34 pixels and exact camera center (principal point) at (160, 120).
     **/
    const unsigned int cam_width  = 320;
    const unsigned int cam_height = 240;
    const float        cam_fx     = 257.34;
    const float        cam_cx     = 160;
    const float        cam_fy     = 257.34;
    const float        cam_cy     = 120;


    /**
     * Next, we need a mesh file!
     * For example, we may want to superimpose the good ol' fiend of Space
     * Invader arcade game.
     * Here, by chance, we have a mesh of it ready to for you!
     *
     * To associate a mesh to a particular object the SICAD::ModelPathContainer
     * comes into play. In this way we can associate a tag to a mesh model and
     * use it during rendering to assign to it a particular pose (position and
     * orientation).
     *
     * NOTE: supported mesh format are the one provided by the ASSIMP library.
     *       Have a look here: http://assimp.org/main_features_formats.html
     **/
    SICAD::ModelPathContainer obj;
    obj.emplace("alien", "./spaceinvader.obj");


    /**
     * We create a SICAD object by just passing all the camera parameters.
     **/
    SICAD si_cad(obj, cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy);


    /**
     * We are close to getting our superimposed mesh!
     * We now need to choose a pose (position and orientation) to which we
     * render the Space Invader fiend.
     * Suppose we want it right in front of us, 10 cm away from the camera.
     * Note that the OpenGL convention is right handed, expressed in meters,
     * with the z axis coming out from the screen.
     * This imply that we want the alien at -0.10 m on the z axis.
     * As far as the orientation is concerned we want it facing us.
     * Since we have to pass an axis-angle vector and that all zeros is invalid,
     * we can just pass any versor with a 0 degree angle.
     *
     * We now have to associate the pose with a particular mesh model.
     * Do you remember that we used "alien" for the Space Invader fiend?
     * Then we just have to associate a tag to the pose and we are ready to
     * render!
     * To do so, we just use Superimpose::ModelPose and
     * Superimpose::ModelPoseContainer as follows.
     **/
    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -0.1;
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;

    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("alien", obj_pose);


    /**
     * Finally we trivially set the pose of the camera as follows.
     *
     * Q: why don't we have another Superimpose:: object to do this?
     * W: well...it's under development!
     **/
    double cam_x[] = {  0, 0, 0};
    double cam_o[] = {1.0, 0, 0, 0};


    /**
     * It's render time!
     * We save the output of the render right into a cv::Mat and we can use
     * the well known OpenCV facilities to do whatever we want with it, for
     * example write it on the filesystem.
     **/
    cv::Mat img;
    si_cad.superimpose(objpose_map, cam_x, cam_o, img);
    cv::imwrite("./spaceinvader.jpg", img);

    return EXIT_SUCCESS;
}
