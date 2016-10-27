#include "SuperimposeHandSkeletonThread.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#define PROJECT_NAME ConstString("superimpose_hand")

using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


SuperimposeHandSkeletonThread::SuperimposeHandSkeletonThread(const ConstString &laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver) :
    log_ID("[SuperimposeHandSkeletonThread]"), laterality(laterality), camera(camera), camsel((camera == "left")? 0:1), arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver) {}


bool SuperimposeHandSkeletonThread::threadInit() {
    yInfo() << log_ID << "Initializing hand skeleton drawing thread.";

    yInfo() << log_ID << "Setting interfaces";

    IControlLimits *itf_fingers_lim;
    arm_remote_driver.view(itf_fingers_lim);
    if (!itf_fingers_lim) {
        yError() << log_ID << "Error getting IControlLimits interface in thread.";
        return false;
    }

    arm_remote_driver.view(itf_arm_encoders);
    if (!itf_arm_encoders) {
        yError() << log_ID << "Error getting IEncoders interface.";
        return false;
    }
    itf_arm_encoders->getAxes(&num_arm_enc);

    arm_cartesian_driver.view(itf_arm_cart);
    if (!itf_arm_cart) {
        yError() << log_ID << "Error getting ICartesianControl interface in thread.";
        return false;
    }

    gaze_driver.view(itf_head_gaze);
    if (!itf_head_gaze) {
        yError() << log_ID << "Error getting IGazeControl interface.";
        return false;
    }

    yInfo() << log_ID << "Interfaces set!";

    yInfo() << log_ID << "Setting joint bounds for the fingers.";

    finger[0] = iCubFinger(laterality+"_thumb");
    finger[1] = iCubFinger(laterality+"_index");
    finger[2] = iCubFinger(laterality+"_middle");

    std::deque<IControlLimits*> temp_lim;
    temp_lim.push_front(itf_fingers_lim);
    for (int i = 0; i < 3; ++i) {
        if (!finger[i].alignJointsBounds(temp_lim)) {
            yError() << log_ID << "Cannot set joint bound for finger " + std::to_string(i) + ".";
            return false;
        }
    }

    yInfo() << log_ID << "Joint bound for finger set!";

    yInfo() << log_ID << "Opening ports for skeleton images.";

    if (!inport_skeleton_img.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+":i")) {
        yError() << log_ID << "Cannot open input image port for "+camera+".";
        return false;
    }

    if (!outport_skeleton_img.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+":o")) {
        yError() << log_ID << "Cannot open output image port for "+camera+".";
        return false;
    }

    yInfo() << log_ID << "Skeleton image ports succesfully opened!";

    yInfo() << log_ID << "Opening ports for "+camera+" camera pose.";

    if (!port_cam_pose.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera+"/pose:o")) {
        yError() << log_ID << "Cannot open "+camera+" camera pose output port.";
        return false;
    }

    yInfo() << log_ID << "Port for "+camera+" camera succesfully opened!";

    yInfo() << log_ID << "Initialization completed!";

    return true;
}


void SuperimposeHandSkeletonThread::run() {
    Vector ee_x(3);
    Vector ee_o(4);
    Vector cam_x(3);
    Vector cam_o(4);

    while (!isStopping()) {
        ImageOf<PixelRgb> *imgin = inport_skeleton_img.read(true);
        itf_arm_cart->getPose(ee_x, ee_o);
        itf_head_gaze->getLeftEyePose(cam_x, cam_o);

        if (imgin != NULL) {
            ImageOf<PixelRgb> &imgout = outport_skeleton_img.prepare();
            imgout = *imgin;

            cv::Mat img = cv::cvarrToMat(imgout.getIplImage());

            Matrix Ha = axis2dcm(ee_o);
            ee_x.push_back(1.0);
            Ha.setCol(3, ee_x);

            Vector endeffector_pixel;
            itf_head_gaze->get2DPixel(camsel, ee_x, endeffector_pixel);

            cv::Point endeffector_point(static_cast<int>(endeffector_pixel[0]), static_cast<int>(endeffector_pixel[1]));
            cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

            Vector encs(static_cast<size_t>(num_arm_enc));
            Vector chainjoints;
            itf_arm_encoders->getEncoders(encs.data());
            for (unsigned int i = 0; i < 3; ++i) {
                finger[i].getChainJoints(encs, chainjoints);
                finger[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            for (unsigned int fng = 0; fng < 3; ++fng) {
                std::deque<cv::Point> current_joint_point;

                Vector current_joint_pixel;
                itf_head_gaze->get2DPixel(camsel, Ha*(finger[fng].getH0().getCol(3)), current_joint_pixel);

                current_joint_point.push_front(cv::Point(static_cast<int>(current_joint_pixel[0]), static_cast<int>(current_joint_pixel[1])));
                cv::circle(img, current_joint_point.front(), 3, cv::Scalar(0, 0, 255), 4);

                cv::line(img, endeffector_point, current_joint_point.front(), cv::Scalar(255, 0, 0), 2);

                for (unsigned int i = 0; i < finger[fng].getN(); ++i) {
                    Vector current_joint_pixel;
                    itf_head_gaze->get2DPixel(camsel, Ha*(finger[fng].getH(i, true).getCol(3)), current_joint_pixel);

                    current_joint_point.push_front(cv::Point(static_cast<int>(current_joint_pixel[0]), static_cast<int>(current_joint_pixel[1])));
                    cv::circle(img, current_joint_point.front(), 3, cv::Scalar(0, 0, 255), 4);

                    cv::line(img, current_joint_point.front(), current_joint_point.back(), cv::Scalar(255, 255, 255), 2);
                    current_joint_point.pop_back();
                }
            }

            Bottle &camPoseBottle = port_cam_pose.prepare();
            camPoseBottle.clear();
            camPoseBottle.addString(cam_x.toString() + "    " + cam_o.toString());

            outport_skeleton_img.write();
            port_cam_pose.write();
        }
    }
}


void SuperimposeHandSkeletonThread::onStop() {
    inport_skeleton_img.interrupt();
}


void SuperimposeHandSkeletonThread::threadRelease() {
    yInfo() << log_ID << "Deallocating resource of hand skeleton drawing thread.";

    outport_skeleton_img.interrupt();
    port_cam_pose.interrupt();

    if (!inport_skeleton_img.isClosed())  inport_skeleton_img.close();
    if (!outport_skeleton_img.isClosed()) outport_skeleton_img.close();
    if (!port_cam_pose.isClosed())        port_cam_pose.close();

    yInfo() << log_ID << "Deallocation completed!";
}
