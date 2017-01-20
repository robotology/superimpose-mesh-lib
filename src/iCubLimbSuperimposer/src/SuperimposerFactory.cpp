#include "SuperimposerFactory.h"

#include <list>

#include <yarp/os/LogStream.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


bool SuperimposerFactory::FileFound (const ConstString & file)
{
    if (file.empty()) {
        yError() << log_ID_ << "File not found!";
        return false;
    }
    return true;
}


bool SuperimposerFactory::setRightArmRemoteControlboard()
{
    Property rightarm_remote_options;
    rightarm_remote_options.put("device", "remote_controlboard");
    rightarm_remote_options.put("local", "/"+project_name_+"/control_right_arm");
    rightarm_remote_options.put("remote", "/"+robot_+"/right_arm");

    rightarm_remote_driver_.open(rightarm_remote_options);
    if (rightarm_remote_driver_.isValid()) {
        yInfo() << log_ID_ << "Right arm remote_controlboard succefully opened.";

        rightarm_remote_driver_.view(itf_rightarm_enc_);
        if (!itf_rightarm_enc_) {
            yError() << log_ID_ << "Error getting right arm IEncoders interface.\n";
            return false;
        }
        num_rightarm_enc_ = 0;
        itf_rightarm_enc_->getAxes(&num_rightarm_enc_);
        yInfo() << log_ID_ << "Right arm encorders succefully read.";

        rightarm_remote_driver_.view(itf_rightarm_pos_);
        if (!itf_rightarm_pos_) {
            yError() << log_ID_ << "Error getting right arm IPositionControl2 interface.\n";
            return false;
        }
        yInfo() << log_ID_ << "Right arm positions succefully read.";
    } else {
        yError() << log_ID_ << "Error opening right arm remote_controlboard device.\n";
        return false;
    }

    return true;
}


bool SuperimposerFactory::setRightArmCartesianController()
{
    Property rightarm_cartesian_options;
    rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
    rightarm_cartesian_options.put("local", "/"+project_name_+"/cart_right_arm");
    rightarm_cartesian_options.put("remote", "/"+robot_+"/cartesianController/right_arm");

    rightarm_cartesian_driver_.open(rightarm_cartesian_options);
    if (rightarm_cartesian_driver_.isValid()) {
        rightarm_cartesian_driver_.view(itf_rightarm_cart_);
        if (!itf_rightarm_cart_) {
            yError() << log_ID_ << "Error getting ICartesianControl interface.\n";
            return false;
        }
        yInfo() << log_ID_ << "cartesiancontrollerclient succefully opened.";
    } else {
        yError() << log_ID_ << "Error opening cartesiancontrollerclient device.\n";
        return false;
    }

    if (!itf_rightarm_cart_->setTrajTime(10.0))
    {
        yError() << log_ID_ << "Error setting ICartesianControl trajectory time.\n";
        return false;
    }
    yInfo() << log_ID_ << "Succesfully set ICartesianControl trajectory time!\n";

    if(!itf_rightarm_cart_->setInTargetTol(0.01))
    {
        yError() << log_ID_ << "Error setting ICartesianControl target tolerance.\n";
        return false;
    }
    yInfo() << log_ID_ << "Succesfully set ICartesianControl target tolerance!\n";

    return true;
}


bool SuperimposerFactory::setHeadRemoteControlboard()
{
    Property head_option;
    head_option.put("device", "remote_controlboard");
    head_option.put("local", "/"+project_name_+"/control_head");
    head_option.put("remote", "/"+robot_+"/head");

    head_remote_driver_.open(head_option);
    if (head_remote_driver_.isValid()) {
        yInfo() << log_ID_ << "Head remote_controlboard succefully opened.";

        head_remote_driver_.view(itf_head_pos_);
        if (!itf_head_pos_) {
            yError() << log_ID_ << "Error getting head IPositionControl interface.\n";
            return false;
        }
        yInfo() << log_ID_ << "Head positions succefully read.";
    } else {
        yError() << log_ID_ << "Error opening head remote_controlboard device.";
        return false;
    }

    return true;
}


bool SuperimposerFactory::setGazeController()
{
    Property gaze_option;
    gaze_option.put("device", "gazecontrollerclient");
    gaze_option.put("local", "/"+project_name_+"/gaze");
    gaze_option.put("remote", "/iKinGazeCtrl");

    gaze_driver_.open(gaze_option);
    if (gaze_driver_.isValid()) {
        gaze_driver_.view(itf_head_gaze_);
        if (!itf_head_gaze_) {
            yError() << log_ID_ << "Error getting IGazeControl interface.\n";
            return false;
        }
    } else {
        yError() << log_ID_ << "Gaze control device not available.\n";
        return false;
    }

    return true;
}


bool SuperimposerFactory::setTorsoDOF()
{
    Vector curDOF;
    itf_rightarm_cart_->getDOF(curDOF);
    yInfo() << log_ID_ << "Old DOF: [" + curDOF.toString(0) + "].";
    yInfo() << log_ID_ << "Setting iCub to use the DOF from the torso.";
    Vector newDOF(curDOF);
    newDOF[0] = 1;
    newDOF[1] = 2;
    newDOF[2] = 1;
    if (!itf_rightarm_cart_->setDOF(newDOF, curDOF)) {
        yError() << log_ID_ << "Cannot use torso DOF.";
        return false;
    }
    yInfo() << log_ID_ << "Setting the DOF done.";
    yInfo() << log_ID_ << "New DOF: [" + curDOF.toString(0) + "]";

    return true;
}


bool SuperimposerFactory::setCommandPort()
{
    yInfo() << log_ID_ << "Opening command port.";
    if (!port_command_.open("/"+project_name_+"/cmd")) {
        yError() << log_ID_ << "Cannot open the command port.";
        return false;
    }
    if (!this->yarp().attachAsServer(port_command_)) {
        yError() << log_ID_ << "Cannot attach the command port.";
        return false;
    }
    yInfo() << log_ID_ << "Command port and helper succesfully opened and attached. Ready to recieve commands.";

    return true;
}


bool SuperimposerFactory::MoveFingers(const double (&joint)[6])
{
    /* Close iCub hand. */
    yInfo() << log_ID_ << "Closing fingers.";
    Vector rightarm_encoder(static_cast<size_t>(num_rightarm_enc_));
    itf_rightarm_enc_->getEncoders(rightarm_encoder.data());
    std::list<std::pair<unsigned int, double>> joint_pos_map = {{13, joint[0]},
                                                                {14, joint[1]},
                                                                {15, joint[2]},
                                                                { 8, joint[3]},
                                                                { 9, joint[4]},
                                                                {10, joint[5]}};
    for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map) {
        yInfo() << log_ID_ << "Moving joint "+std::to_string(map->first)+" to the position "+std::to_string(map->second)+".";
        if (std::abs(rightarm_encoder[map->first] - map->second) > 5.0) {
            rightarm_encoder[map->first] = map->second;
            itf_rightarm_pos_->positionMove(rightarm_encoder.data());
            Time::delay(2.0);
        }
    }
    yInfo() << log_ID_ << "Fingers succesfully closed.";

    return true;
}


bool SuperimposerFactory::MoveHand(const Matrix &R, const Vector &init_x)
{
    /* Setting hand pose */
    yInfo() << log_ID_ << "Moving hand to the initial position.";
    yInfo() << log_ID_ << "R = " << R.toString();
    yInfo() << log_ID_ << "x = " << init_x.toString();

    Vector init_o(dcm2axis(R));

    itf_rightarm_cart_->goToPoseSync(init_x, init_o);
    itf_rightarm_cart_->waitMotionDone(0.2, 15.0);

    yInfo() << log_ID_ << "The hand is in position.";

    /* Set fixation point */
    Vector tmp;
    itf_head_gaze_->getFixationPoint(tmp);
    Vector init_fixation(init_x);
    init_fixation[0] -= 0.05;
    init_fixation[1] -= 0.05;
    if (norm(tmp - init_fixation) > 0.10) {
        yInfo() << log_ID_ << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
        itf_head_gaze_->lookAtFixationPoint(init_fixation);
        itf_head_gaze_->waitMotionDone(0.1, 15.0);
    }
    yInfo() << log_ID_ << "Gaze motion done.";

    return true;
}


bool SuperimposerFactory::move_hand()
{
    if (!init_position_) {
        yInfo() << log_ID_ << "Starting single hand motion.";

        start_ = true;

        return true;
    } else {
        yWarning() << log_ID_ << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

        return false;
    }
}


bool SuperimposerFactory::move_hand_freerun()
{
    if (!init_position_) {
        yInfo() << log_ID_ << "Starting freerun hand motion.";

        start_ = true;
        freerunning_ = true;

        return true;
    } else {
        yWarning() << log_ID_ << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

        return false;
    }
}


bool SuperimposerFactory::stop_hand()
{
    yInfo() << log_ID_ << "Stopping hand motion when reaching the initial position.";

    start_ = false;
    if (freerunning_) freerunning_ = false;

    return true;
}


bool SuperimposerFactory::initial_position()
{
    if (!init_position_) {
        yWarning() << log_ID_ << "Already in initial position settings!";

        return false;
    } else {
        yInfo() << log_ID_ << "Reaching initial position...";

        init_position_ = !MoveHand(table_view_R_, table_view_x_);
        if (!init_position_) yInfo() << log_ID_ << "...done. iCub can move the hand in this settings.";
        else yWarning() << log_ID_ << "...could not reach initial position!";

        return init_position_;
    }
}


bool SuperimposerFactory::view_hand()
{
    if (!start_) {
        yInfo() << log_ID_ << "Reaching a position close to iCub left camera with the right hand...";

        init_position_ = MoveHand(frontal_view_R_, frontal_view_x_);
        if (!init_position_) yWarning() << log_ID_ << "...could not reach the desired position!";
        else yInfo() << log_ID_ << "...done. iCub can't move the hand in this settings.";

        return init_position_;
    } else {
        yWarning() << log_ID_ << "Can't move hand while moving it!";

        return false;
    }
}


bool SuperimposerFactory::open_fingers()
{
    yInfo() << log_ID_ << "Opening fingers...";

    bool motion_done = MoveFingers(open_hand_joints_);
    if (!motion_done) yWarning() << log_ID_ << "...fingers could not be opened!";
    else yInfo() << log_ID_ << "...done.";

    return motion_done;
}


bool SuperimposerFactory::close_fingers()
{
    yInfo() << log_ID_ << "Closing fingers...";

    bool motion_done = MoveFingers(closed_hand_joints_);
    if (!motion_done) yWarning() << log_ID_ << "...fingers could not be closed!";
    else yInfo() << log_ID_ << "...done.";

    return motion_done;
}


bool SuperimposerFactory::view_skeleton(const bool status)
{
    if (!superimpose_skeleton_ && status) {
        trd_left_cam_skeleton_ = new SkeletonSuperimposer(project_name_, "right", "left", rightarm_remote_driver_, rightarm_cartesian_driver_, gaze_driver_);

        if (trd_left_cam_skeleton_ != NULL) {
            yInfo() << log_ID_ << "Starting skeleton superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_skeleton_->start()) {
                yWarning() << log_ID_ << "...thread could not be started!";

                superimpose_skeleton_ = false;
            } else {
                yInfo() << log_ID_ << "...done.";

                superimpose_skeleton_ = true;
            }
        } else {
            yWarning() << log_ID_ << "Could not initialize hand skeleton superimposition!";

            superimpose_skeleton_ = false;
        }

        return superimpose_skeleton_;

    } else if (superimpose_skeleton_ && !status) {
        yInfo() << log_ID_ << "Stopping hand skeleton superimposing thread for the right hand on the left camera images...";

        if (!trd_left_cam_skeleton_->stop()) {
            yWarning() << log_ID_ << "...thread could not be stopped!";

            superimpose_skeleton_ = true;
        } else {
            yInfo() << log_ID_ << "...done.";

            delete trd_left_cam_skeleton_;
            trd_left_cam_skeleton_ = nullptr;

            superimpose_skeleton_ = false;
        }

        return !superimpose_skeleton_;

    } else return false;
}


bool SuperimposerFactory::view_mesh(const bool status) {
    if (!superimpose_mesh_ && status) {
        trd_left_cam_cad_ = new CADSuperimposer(project_name_, "right", "left", rightarm_remote_driver_, rightarm_cartesian_driver_, gaze_driver_, cad_hand_, window_);
        if (trd_left_cam_cad_ != NULL) {
            yInfo() << log_ID_ << "Starting mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_cad_->start()) {
                yWarning() << log_ID_ << "...thread could not be started!";

                superimpose_mesh_ = false;
            } else {
                yInfo() << log_ID_ << "...done.";

                superimpose_mesh_ = true;
            }
        } else {
            yWarning() << log_ID_ << "Could not initialize hand mesh superimposition!";

            superimpose_mesh_ = false;
        }

        return superimpose_mesh_;

    } else if (superimpose_mesh_ && !status) {
        yInfo() << log_ID_ << "Stopping hand mesh superimposing thread for the right hand on the left camera images...";

        if (!trd_left_cam_cad_->stop()) {
            yWarning() << log_ID_ << "...thread could not be stopped!";

            superimpose_mesh_ = true;
        } else {
            yInfo() << log_ID_ << "...done.";

            delete trd_left_cam_cad_;
            trd_left_cam_cad_ = nullptr;

            superimpose_mesh_ = false;
        }

        return !superimpose_mesh_;

    } else return false;
}


std::string SuperimposerFactory::quit() {
    yInfo() << log_ID_ << "Quitting...";

    this->stopModule();

    return "[bye]";
}


SuperimposerFactory::SuperimposerFactory(const yarp::os::ConstString & project_name) : log_ID_("[SuperimposerFactory]"), project_name_(project_name) {}


SuperimposerFactory::SuperimposerFactory() : log_ID_("[SuperimposerFactory]"), project_name_("SuperimposerModule") {}


bool SuperimposerFactory::configure(ResourceFinder &rf)
{
    this->setName(project_name_.c_str());

    /* Setting default parameters. */
    start_ = false;
    init_position_ = false;
    freerunning_ = false;
    superimpose_skeleton_ = false;
    superimpose_mesh_ = false;

    /* Parsing parameters from config file. */
    /* Robot name */
    robot_ = rf.findGroup("PARAMETER").check("robot", Value("icub")).asString();

    /* Joint velocities/accelerations */
    if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16)
    {
        Vector arm_vel(16);
        for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i)
        {
            arm_vel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
        }
        yWarning() << log_ID_ << "Unused config velocities!";
        yInfo()    << log_ID_ << arm_vel.toString();

        Vector arm_acc(16);
        for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("acc").tail().size(); ++i)
        {
            arm_acc[i] = rf.findGroup("ARMJOINT").findGroup("acc").tail().get(i).asDouble();
        }
        yWarning() << log_ID_ << "Unused config accelerations!";
        yInfo()    << log_ID_ << arm_acc.toString();
    }

    cad_hand_["palm"] = rf.findFileByName("r_palm.obj");
    if (!FileFound(cad_hand_["palm"])) return false;
    cad_hand_["thumb1"] = rf.findFileByName("r_tl0.obj");
    if (!FileFound(cad_hand_["thumb1"])) return false;
    cad_hand_["thumb2"] = rf.findFileByName("r_tl1.obj");
    if (!FileFound(cad_hand_["thumb2"])) return false;
    cad_hand_["thumb3"] = rf.findFileByName("r_tl2.obj");
    if (!FileFound(cad_hand_["thumb3"])) return false;
    cad_hand_["thumb4"] = rf.findFileByName("r_tl3.obj");
    if (!FileFound(cad_hand_["thumb4"])) return false;
    cad_hand_["thumb5"] = rf.findFileByName("r_tl4.obj");
    if (!FileFound(cad_hand_["thumb5"])) return false;
    cad_hand_["index0"] = rf.findFileByName("r_indexbase.obj");
    if (!FileFound(cad_hand_["index0"])) return false;
    cad_hand_["index1"] = rf.findFileByName("r_ail0.obj");
    if (!FileFound(cad_hand_["index1"])) return false;
    cad_hand_["index2"] = rf.findFileByName("r_ail1.obj");
    if (!FileFound(cad_hand_["index2"])) return false;
    cad_hand_["index3"] = rf.findFileByName("r_ail2.obj");
    if (!FileFound(cad_hand_["index3"])) return false;
    cad_hand_["index4"] = rf.findFileByName("r_ail3.obj");
    if (!FileFound(cad_hand_["index4"])) return false;
    cad_hand_["medium0"] = rf.findFileByName("r_ml0.obj");
    if (!FileFound(cad_hand_["medium0"])) return false;
    cad_hand_["medium1"] = rf.findFileByName("r_ml1.obj");
    if (!FileFound(cad_hand_["medium1"])) return false;
    cad_hand_["medium2"] = rf.findFileByName("r_ml2.obj");
    if (!FileFound(cad_hand_["medium2"])) return false;
    cad_hand_["medium3"] = rf.findFileByName("r_ml3.obj");
    if (!FileFound(cad_hand_["medium3"])) return false;

    /* Initializing useful pose matrices and vectors for the hand. */
    frontal_view_R_.resize(3, 3);
    frontal_view_R_(0,0) =  0.0;   frontal_view_R_(0,1) =  0.0;   frontal_view_R_(0,2) =  1.0;
    frontal_view_R_(1,0) = -1.0;   frontal_view_R_(1,1) =  0.0;   frontal_view_R_(1,2) =  0.0;
    frontal_view_R_(2,0) =  0.0;   frontal_view_R_(2,1) = -1.0;   frontal_view_R_(2,2) =  0.0;

    frontal_view_x_.resize(3);
    frontal_view_x_[0] = -0.25;
    frontal_view_x_[1] = +0.00;
    frontal_view_x_[2] = +0.20;

    table_view_R_.resize(3, 3);
    table_view_R_(0,0) = -1.0;   table_view_R_(0,1) =  0.0;   table_view_R_(0,2) =  0.0;
    table_view_R_(1,0) =  0.0;   table_view_R_(1,1) =  1.0;   table_view_R_(1,2) =  0.0;
    table_view_R_(2,0) =  0.0;   table_view_R_(2,1) =  0.0;   table_view_R_(2,2) = -1.0;

    table_view_x_.resize(3);
    table_view_x_[0] = -0.40;
    table_view_x_[1] = +0.10;
    table_view_x_[2] = +0.10;

    open_hand_joints_[0] = 0;
    open_hand_joints_[1] = 0;
    open_hand_joints_[2] = 0;
    open_hand_joints_[3] = 10;
    open_hand_joints_[4] = 0;
    open_hand_joints_[5] = 0;

    closed_hand_joints_[0] = 80;
    closed_hand_joints_[1] = 150;
    closed_hand_joints_[2] = 180;
    closed_hand_joints_[3] = 80;
    closed_hand_joints_[4] = 10;
    closed_hand_joints_[5] = 80;

    /* Right arm control board. */
    if (!setRightArmRemoteControlboard()) return false;

    /* Right arm cartesian controler. */
    if (!setRightArmCartesianController()) return false;

    /* Head control board. */
    if (!setHeadRemoteControlboard()) return false;

    /* Gaze control. */
    if (!setGazeController()) return false;

    /* Enable torso DOF. */
    if (!setTorsoDOF()) return false;

    /* Set default right hand configuration (closed). */
    if (!MoveFingers(open_hand_joints_)) return false;

    /* Set deafult initial pose of the hand (table view). */
    if (!MoveHand(table_view_R_, table_view_x_)) return false;

    /* Set initial finger motion point */
    radius_      = 0.08;
    angle_ratio_ = 12;
    motion_time_ = 10.0;
    path_time_   = motion_time_ / angle_ratio_;

    /* Open a remote command port and allow the program be started */
    return setCommandPort();
}


bool SuperimposerFactory::updateModule()
{
    if (start_)
    {
        Vector motion_axis;
        Vector motion_angle;
        Vector center(2);
        double old_traj_time;

        itf_rightarm_cart_->getTrajTime(&old_traj_time);
        itf_rightarm_cart_->setTrajTime(path_time_);

        itf_rightarm_cart_->getPose(motion_axis, motion_angle);
        center[0] = motion_axis[0];
        center[1] = motion_axis[1] - radius_;

        yInfo() << log_ID_ << "Starting finger motion.";
        for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angle_ratio_)
        {
            motion_axis[0] = (center[0] - (radius_ * sin(alpha)));
            motion_axis[1] = (center[1] + (radius_ * cos(alpha)));
            yInfo() << log_ID_ << "Next position: [" << motion_axis.toString() << "].";
            itf_rightarm_cart_->goToPose(motion_axis, motion_angle);
            Time::delay(0.7 * path_time_);
        }
        yInfo() << log_ID_ << "Motion done.";
        if (!freerunning_) start_ = false;

        itf_rightarm_cart_->setTrajTime(old_traj_time);
    }
    return true;
}


bool SuperimposerFactory::close()
{
    yInfo() << log_ID_ << "Calling close functions...";

    if (superimpose_skeleton_) trd_left_cam_skeleton_->stop();
    if (superimpose_mesh_)     trd_left_cam_cad_->stop();

    delete trd_left_cam_skeleton_;
    delete trd_left_cam_cad_;
    
    itf_rightarm_cart_->removeTipFrame();
    
    if (rightarm_cartesian_driver_.isValid()) rightarm_cartesian_driver_.close();
    if (rightarm_remote_driver_.isValid())    rightarm_remote_driver_.close();
    if (head_remote_driver_.isValid())        head_remote_driver_.close();
    if (gaze_driver_.isValid())               gaze_driver_.close();

    if (port_command_.isOpen()) port_command_.close();
    return true;
}
