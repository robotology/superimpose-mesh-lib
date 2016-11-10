#include "SkeletonSuperimposer.h"

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


SkeletonSuperimposer::SkeletonSuperimposer(const ConstString & laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver) : log_ID_("[SkeletonSuperimposer]"), laterality_(laterality), camera_(camera), camsel_((camera == "left")? 0:1), arm_remote_driver_(arm_remote_driver), arm_cartesian_driver_(arm_cartesian_driver), gaze_driver_(gaze_driver) {}


bool SkeletonSuperimposer::threadInit() {
    yInfo() << log_ID_ << "Initializing hand skeleton drawing thread.";

    yInfo() << log_ID_ << "Setting interfaces";

    IControlLimits *itf_fingers_lim;
    arm_remote_driver_.view(itf_fingers_lim);
    if (!itf_fingers_lim)
    {
        yError() << log_ID_ << "Error getting IControlLimits interface in thread.";
        return false;
    }

    arm_remote_driver_.view(itf_arm_encoders_);
    if (!itf_arm_encoders_)
    {
        yError() << log_ID_ << "Error getting IEncoders interface.";
        return false;
    }
    itf_arm_encoders_->getAxes(&num_arm_enc_);

    arm_cartesian_driver_.view(itf_arm_cart_);
    if (!itf_arm_cart_)
    {
        yError() << log_ID_ << "Error getting ICartesianControl interface in thread.";
        return false;
    }

    gaze_driver_.view(itf_head_gaze_);
    if (!itf_head_gaze_)
    {
        yError() << log_ID_ << "Error getting IGazeControl interface.";
        return false;
    }

    yInfo() << log_ID_ << "Interfaces set!";

    Bottle btl_cam_left_info;
    itf_head_gaze_->getInfo(btl_cam_left_info);
    Bottle * cam_left_info = btl_cam_left_info.findGroup("camera_intrinsics_left").get(1).asList();
    yInfo() << log_ID_ << "Camera Info: [" + cam_left_info->toString() + "].";
    EYE_FX_ = static_cast<float>(cam_left_info->get(0).asDouble());
    EYE_CX_ = static_cast<float>(cam_left_info->get(2).asDouble());
    EYE_FY_ = static_cast<float>(cam_left_info->get(5).asDouble());
    EYE_CY_ = static_cast<float>(cam_left_info->get(6).asDouble());

    yInfo() << log_ID_ << "Setting joint bounds for the fingers.";

    finger_[0] = iCubFinger(laterality_+"_thumb");
    finger_[1] = iCubFinger(laterality_+"_index");
    finger_[2] = iCubFinger(laterality_+"_middle");

    std::deque<IControlLimits*> temp_lim;
    temp_lim.push_front(itf_fingers_lim);
    for (int i = 0; i < 3; ++i)
    {
        if (!finger_[i].alignJointsBounds(temp_lim))
        {
            yError() << log_ID_ << "Cannot set joint bound for finger " + std::to_string(i) + ".";
            return false;
        }
    }

    yInfo() << log_ID_ << "Joint bound for finger set!";

    yInfo() << log_ID_ << "Opening ports for skeleton images.";

    if (!inport_skeleton_img_.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera_+":i"))
    {
        yError() << log_ID_ << "Cannot open input image port for "+camera_+".";
        return false;
    }

    if (!outport_skeleton_img_.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera_+":o"))
    {
        yError() << log_ID_ << "Cannot open output image port for "+camera_+".";
        return false;
    }

    yInfo() << log_ID_ << "Skeleton image ports succesfully opened!";

    yInfo() << log_ID_ << "Opening ports for "+camera_+" camera_ pose.";

    if (!port_cam_pose_.open("/"+PROJECT_NAME+"/skeleton/cam/"+camera_+"/pose:o"))
    {
        yError() << log_ID_ << "Cannot open "+camera_+" camera_ pose output port.";
        return false;
    }

    yInfo() << log_ID_ << "Port for "+camera_+" camera_ succesfully opened!";

    yInfo() << log_ID_ << "Setting up OpenCV drawer...";

    drawer_.Configure(EYE_FX_, EYE_FY_, EYE_CX_, EYE_CY_);

    yInfo() << log_ID_ << "OpenCV drawer succesfully set!";

    yInfo() << log_ID_ << "Initialization completed!";

    return true;
}

// TODO: uniformare il nome delle parti da disegnare così da avere dei Superimposer standardizzati a cui passare un generico SuperImpose
// TODO: trovare un modo per avere dei configure nell'interfaccia SuperImpose così da poter usare un factory method per i vari Superimposer
void SkeletonSuperimposer::run() {
    Vector ee_x(3);
    Vector ee_o(4);
    Vector cam_x(3);
    Vector cam_o(4);

    while (!isStopping()) {
        ImageOf<PixelRgb> * imgin = inport_skeleton_img_.read(true);

        itf_arm_cart_->getPose(ee_x, ee_o);

        itf_head_gaze_->getLeftEyePose(cam_x, cam_o);

        Matrix Ha = axis2dcm(ee_o);
        ee_x.push_back(1.0);
        Ha.setCol(3, ee_x);

        Vector encs(static_cast<size_t>(num_arm_enc_));
        Vector chainjoints;
        itf_arm_encoders_->getEncoders(encs.data());
        for (unsigned int i = 0; i < 3; ++i)
        {
            finger_[i].getChainJoints(encs, chainjoints);
            finger_[i].setAng(CTRL_DEG2RAD * chainjoints);
        }

        SuperImpose::ObjPoseMap hand_pose;
        SuperImpose::ObjPose    pose;
        pose.assign(ee_x.data(), ee_x.data()+3);
        pose.insert(pose.end(), ee_o.data(), ee_o.data()+4);
        hand_pose.emplace("palm", pose);
        for (unsigned int fng = 0; fng < 3; ++fng)
        {
            std::string finger_s;
            pose.clear();
            if (fng != 0)
            {
                Vector j_x = (Ha * (finger_[fng].getH0().getCol(3))).subVector(0, 2);
                Vector j_o = dcm2axis(Ha * finger_[fng].getH0());

                if      (fng == 1) { finger_s = "index"; }
                else if (fng == 2) { finger_s = "medium"; }

                pose.assign(j_x.data(), j_x.data()+3);
                pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                hand_pose.emplace(finger_s, pose);
            }

            for (unsigned int i = 0; i < finger_[fng].getN(); ++i)
            {
                Vector j_x = (Ha * (finger_[fng].getH(i, true).getCol(3))).subVector(0, 2);
                Vector j_o = dcm2axis(Ha * finger_[fng].getH(i, true));

                if      (fng == 0) { finger_s = "thumb"; }
                else if (fng == 1) { finger_s = "index"; }
                else if (fng == 2) { finger_s = "medium"; }

                pose.assign(j_x.data(), j_x.data()+3);
                pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                hand_pose.emplace(finger_s, pose);
            }
        }

        if (imgin != NULL) {
            ImageOf<PixelRgb> & imgout = outport_skeleton_img_.prepare();
            imgout = *imgin;
            cv::Mat img = cv::cvarrToMat(imgout.getIplImage());
            drawer_.Superimpose(hand_pose, cam_x.data(), cam_o.data(), img);

            Bottle &camPoseBottle = port_cam_pose_.prepare();
            camPoseBottle.clear();
            camPoseBottle.addString(cam_x.toString() + " " + cam_o.toString());
            
            outport_skeleton_img_.write();
            port_cam_pose_.write();
        }
    }
}


void SkeletonSuperimposer::onStop() {
    inport_skeleton_img_.interrupt();
}


void SkeletonSuperimposer::threadRelease() {
    yInfo() << log_ID_ << "Deallocating resource of hand skeleton drawing thread.";

    outport_skeleton_img_.interrupt();
    port_cam_pose_.interrupt();

    if (!inport_skeleton_img_.isClosed())  inport_skeleton_img_.close();
    if (!outport_skeleton_img_.isClosed()) outport_skeleton_img_.close();
    if (!port_cam_pose_.isClosed())        port_cam_pose_.close();

    yInfo() << log_ID_ << "Deallocation completed!";
}
