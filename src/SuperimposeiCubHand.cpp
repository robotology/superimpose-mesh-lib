#include "SuperimposeiCubHand.h"

#include <list>

#include <yarp/os/LogStream.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#define PROJECT_NAME ConstString("superimpose_hand")

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


bool SuperimposeHand::fileFound (const ConstString &file) {
    if (file.empty()) {
        yError() << log_ID << "File not found!";
        return false;
    }
    return true;
}


bool SuperimposeHand::setRightArmRemoteControlboard()
{
    Property rightarm_remote_options;
    rightarm_remote_options.put("device", "remote_controlboard");
    rightarm_remote_options.put("local", "/"+PROJECT_NAME+"/control_right_arm");
    rightarm_remote_options.put("remote", "/"+robot+"/right_arm");

    rightarm_remote_driver.open(rightarm_remote_options);
    if (rightarm_remote_driver.isValid()) {
        yInfo() << log_ID << "Right arm remote_controlboard succefully opened.";

        rightarm_remote_driver.view(itf_rightarm_enc);
        if (!itf_rightarm_enc) {
            yError() << log_ID << "Error getting right arm IEncoders interface.\n";
            return false;
        }
        num_rightarm_enc = 0;
        itf_rightarm_enc->getAxes(&num_rightarm_enc);
        yInfo() << log_ID << "Right arm encorders succefully read.";

        rightarm_remote_driver.view(itf_rightarm_pos);
        if (!itf_rightarm_pos) {
            yError() << log_ID << "Error getting right arm IPositionControl2 interface.\n";
            return false;
        }
        yInfo() << log_ID << "Right arm positions succefully read.";
    } else {
        yError() << log_ID << "Error opening right arm remote_controlboard device.\n";
        return false;
    }

    num_rightarm_joint = 0;
    itf_rightarm_pos->getAxes(&num_rightarm_joint);
    yInfo() << log_ID << "Total number of right arm joints: " << num_rightarm_joint << ".";
    Vector tmp(static_cast<size_t>(num_rightarm_joint));
    for (int i = 0; i < num_rightarm_joint; ++i) {
        tmp[i] = 10.0;
    }
    if (!itf_rightarm_pos->setRefAccelerations(tmp.data()))
    {
        yError() << log_ID << "Error setting right arm joint accelerations.\n";
        return false;
    }
    for (int i = 0; i < num_rightarm_joint; ++i) {
        tmp[i] = 15.0;
        if (!itf_rightarm_pos->setRefSpeed(i, tmp[i]))
        {
            yError() << log_ID << "Error setting right arm joint speeds.\n";
            return false;
        }
    }
    yInfo() << log_ID << "Right arm joint speeds and accelerations succesfully set.";

    return true;
}


bool SuperimposeHand::setRightArmCartesianController()
{
    Property rightarm_cartesian_options;
    rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
    rightarm_cartesian_options.put("local", "/"+PROJECT_NAME+"/cart_right_arm");
    rightarm_cartesian_options.put("remote", "/"+robot+"/cartesianController/right_arm");

    rightarm_cartesian_driver.open(rightarm_cartesian_options);
    if (rightarm_cartesian_driver.isValid()) {
        rightarm_cartesian_driver.view(itf_rightarm_cart);
        if (!itf_rightarm_cart) {
            yError() << log_ID << "Error getting ICartesianControl interface.\n";
            return false;
        }
        yInfo() << log_ID << "cartesiancontrollerclient succefully opened.";
    } else {
        yError() << log_ID << "Error opening cartesiancontrollerclient device.\n";
        return false;
    }

    return true;
}


bool SuperimposeHand::setHeadRemoteControlboard()
{
    Property head_option;
    head_option.put("device", "remote_controlboard");
    head_option.put("local", "/"+PROJECT_NAME+"/control_head");
    head_option.put("remote", "/"+robot+"/head");

    head_remote_driver.open(head_option);
    if (head_remote_driver.isValid()) {
        yInfo() << log_ID << "Head remote_controlboard succefully opened.";

        head_remote_driver.view(itf_head_pos);
        if (!itf_head_pos) {
            yError() << log_ID << "Error getting head IPositionControl interface.\n";
            return false;
        }
        yInfo() << log_ID << "Head positions succefully read.";
    } else {
        yError() << log_ID << "Error opening head remote_controlboard device.";
        return false;
    }

    num_head_joint = 0;
    itf_head_pos->getAxes(&num_head_joint);
    yInfo() << log_ID << "Total number of head joints: " << num_head_joint << ".";
    Vector tmp(static_cast<size_t>(num_head_joint));
    for (int i = 0; i < num_head_joint; ++i) {
        tmp[i] = 10.0;
    }
    if (!itf_head_pos->setRefAccelerations(tmp.data()))
    {
        yError() << log_ID << "Error setting head joint accelerations.\n";
        return false;
    }
    for (int i = 0; i < num_head_joint; ++i) {
        tmp[i] = 15.0;
        if (!itf_head_pos->setRefSpeed(i, tmp[i]))
        {
            yError() << log_ID << "Error setting head joint speeds.\n";
            return false;
        }
    }
    yInfo() << log_ID << "Head joint speeds and accelerations set.";

    return true;
}


bool SuperimposeHand::setGazeController()
{
    Property gaze_option;
    gaze_option.put("device", "gazecontrollerclient");
    gaze_option.put("local", "/"+PROJECT_NAME+"/gaze");
    gaze_option.put("remote", "/iKinGazeCtrl");

    gaze_driver.open(gaze_option);
    if (gaze_driver.isValid()) {
        gaze_driver.view(itf_head_gaze);
        if (!itf_head_gaze) {
            yError() << log_ID << "Error getting IGazeControl interface.\n";
            return false;
        }
    } else {
        yError() << log_ID << "Gaze control device not available.\n";
        return false;
    }

    return true;
}


bool SuperimposeHand::setTorsoDOF()
{
    Vector curDOF;
    itf_rightarm_cart->getDOF(curDOF);
    yInfo() << log_ID << "Old DOF: [" + curDOF.toString(0) + "].";
    yInfo() << log_ID << "Setting iCub to use the DOF from the torso.";
    Vector newDOF(curDOF);
    newDOF[0] = 1;
    newDOF[1] = 2;
    newDOF[2] = 1;
    if (!itf_rightarm_cart->setDOF(newDOF, curDOF)) {
        yError() << log_ID << "Cannot use torso DOF.";
        return false;
    }
    yInfo() << log_ID << "Setting the DOF done.";
    yInfo() << log_ID << "New DOF: [" + curDOF.toString(0) + "]";

    return true;
}


bool SuperimposeHand::setCommandPort()
{
    yInfo() << log_ID << "Opening command port.";
    if (!port_command.open("/"+PROJECT_NAME+"/cmd")) {
        yError() << log_ID << "Cannot open the command port.";
        return false;
    }
    if (!this->yarp().attachAsServer(port_command)) {
        yError() << log_ID << "Cannot attach the command port.";
        return false;
    }
    yInfo() << log_ID << "Command port succesfully opened and attached. Ready to start and recieve commands.";

    return true;
}


bool SuperimposeHand::moveFingers(const double (&joint)[6])
{
    /* Close iCub hand. */
    yInfo() << log_ID << "Closing fingers.";
    Vector rightarm_encoder(static_cast<size_t>(num_rightarm_enc));
    itf_rightarm_enc->getEncoders(rightarm_encoder.data());
    std::list<std::pair<unsigned int, double>> joint_pos_map = {{13, joint[0]},
        {14, joint[1]},
        {15, joint[2]},
        { 8, joint[3]},
        { 9, joint[4]},
        {10, joint[5]}};
    for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map) {
        yInfo() << log_ID << "Moving joint "+std::to_string(map->first)+" to the position "+std::to_string(map->second)+".";
        if (std::abs(rightarm_encoder[map->first] - map->second) > 5.0) {
            rightarm_encoder[map->first] = map->second;
            itf_rightarm_pos->positionMove(rightarm_encoder.data());
            Time::delay(2.0);
        }
    }
    yInfo() << log_ID << "Fingers succesfully closed.";

    return true;
}


bool SuperimposeHand::moveHand(const Matrix &R, const Vector &init_x)
{
    /* Setting hand pose */
    yInfo() << log_ID << "Moving hand to the initial position.";

    Vector init_o(dcm2axis(R));

    itf_rightarm_cart->goToPoseSync(init_x, init_o);
    itf_rightarm_cart->waitMotionDone(0.1, 6.0);

    yInfo() << log_ID << "The hand is in position.";

    /* Set initial fixation point */
    Vector tmp;
    itf_head_gaze->getFixationPoint(tmp);
    Vector init_fixation(init_x);
    init_fixation[0] -= 0.05;
    init_fixation[1] -= 0.05;
    if (norm(tmp - init_fixation) > 0.10) {
        yInfo() << log_ID << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
        itf_head_gaze->lookAtFixationPoint(init_fixation);
        itf_head_gaze->waitMotionDone(0.1, 6.0);
    }
    yInfo() << log_ID << "Gaze motion done.";

    return true;
}


bool SuperimposeHand::move_hand()
{
    if (!init_position) {
        yInfo() << log_ID << "Starting single hand motion.";

        start = true;

        return true;
    } else {
        yWarning() << log_ID << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

        return false;
    }
}


bool SuperimposeHand::move_hand_freerun()
{
    if (!init_position) {
        yInfo() << log_ID << "Starting freerun hand motion.";

        start = true;
        freerunning = true;

        return true;
    } else {
        yWarning() << log_ID << "Can't move hand in this settings! Use initial_position() before using move_hand() again.";

        return false;
    }
}


bool SuperimposeHand::stop_hand()
{
    yInfo() << log_ID << "Stopping hand motion when reaching the initial position.";

    start = false;
    if (freerunning) freerunning = false;

    return true;
}


bool SuperimposeHand::initial_position()
{
    if (!init_position) {
        yWarning() << log_ID << "Already in initial position settings!";

        return false;
    } else {
        yInfo() << log_ID << "Reaching initial position...";

        init_position = !moveHand(table_view_R, table_view_x);
        if (!init_position) yInfo() << log_ID << "...done. iCub can move the hand in this settings.";
        else yWarning() << log_ID << "...could not reach initial position!";

        return init_position;
    }
}


bool SuperimposeHand::view_hand()
{
    if (!start) {
        yInfo() << log_ID << "Reaching a position close to iCub left camera with the right hand...";

        init_position = moveHand(frontal_view_R, frontal_view_x);
        if (!init_position) yWarning() << log_ID << "...could not reach the desired position!";
        else yInfo() << log_ID << "...done. iCub can't move the hand in this settings.";

        return init_position;
    } else {
        yWarning() << log_ID << "Can't move hand while moving it!";

        return false;
    }
}


bool SuperimposeHand::open_fingers()
{
    yInfo() << log_ID << "Opening fingers...";

    bool motion_done = moveFingers(open_hand_joints);
    if (!motion_done) yWarning() << log_ID << "...fingers could not be opened!";
    else yInfo() << log_ID << "...done.";

    return motion_done;
}


bool SuperimposeHand::close_fingers()
{
    yInfo() << log_ID << "Closing fingers...";

    bool motion_done = moveFingers(closed_hand_joints);
    if (!motion_done) yWarning() << log_ID << "...fingers could not be closed!";
    else yInfo() << log_ID << "...done.";

    return motion_done;
}


bool SuperimposeHand::view_skeleton(const bool status)
{
    if (!superimpose_skeleton && status) {
        trd_left_cam_skeleton = new SuperimposeHandSkeletonThread("right", "left", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver);

        if (trd_left_cam_skeleton != NULL) {
            yInfo() << log_ID << "Starting skeleton superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_skeleton->start()) {
                yWarning() << log_ID << "...thread could not be started!";

                superimpose_skeleton = false;
            } else {
                yInfo() << log_ID << "...done.";

                superimpose_skeleton = true;
            }
        } else {
            yWarning() << log_ID << "Could not initialize hand skeleton superimposition!";

            superimpose_skeleton = false;
        }

        return superimpose_skeleton;

    } else if (superimpose_skeleton && !status) {
        yInfo() << log_ID << "Stopping hand skeleton superimposing thread for the right hand on the left camera images...";

        if (!trd_left_cam_skeleton->stop()) {
            yWarning() << log_ID << "...thread could not be stopped!";

            superimpose_skeleton = true;
        } else {
            yInfo() << log_ID << "...done.";

            delete trd_left_cam_skeleton;
            trd_left_cam_skeleton = nullptr;

            superimpose_skeleton = false;
        }

        return !superimpose_skeleton;

    } else return false;
}


bool SuperimposeHand::view_mesh(const bool status) {
    if (!superimpose_mesh && status) {
        trd_left_cam_cad = new SuperimposeHandCADThread("right", "left", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver, shader_background_vert, shader_background_frag, shader_model_vert, shader_model_frag, cad_hand, window);

        if (trd_left_cam_cad != NULL) {
            yInfo() << log_ID << "Starting mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_cad->start()) {
                yWarning() << log_ID << "...thread could not be started!";

                superimpose_mesh = false;
            } else {
                yInfo() << log_ID << "...done.";

                superimpose_mesh = true;
            }
        } else {
            yWarning() << log_ID << "Could not initialize hand mesh superimposition!";

            superimpose_mesh = false;
        }

        return superimpose_mesh;

    } else if (superimpose_mesh && !status) {
        yInfo() << log_ID << "Stopping hand mesh superimposing thread for the right hand on the left camera images...";

        if (!trd_left_cam_cad->stop()) {
            yWarning() << log_ID << "...thread could not be stopped!";

            superimpose_mesh = true;
        } else {
            yInfo() << log_ID << "...done.";

            delete trd_left_cam_cad;
            trd_left_cam_cad = nullptr;

            superimpose_mesh = false;
        }

        return !superimpose_mesh;

    } else return false;
}


std::string SuperimposeHand::quit() {
    yInfo() << log_ID << "Quitting...";

    this->stopModule();

    return "[bye]";
}


SuperimposeHand::SuperimposeHand() : log_ID("[SuperimposeHand]") {}


bool SuperimposeHand::configure(ResourceFinder &rf)
{
    this->setName(PROJECT_NAME.c_str());

    /* Setting default parameters. */
    start = false;
    init_position = false;
    freerunning = false;
    superimpose_skeleton = false;
    superimpose_mesh = false;

    /* Parsing parameters from config file. */
    robot = rf.findGroup("PARAMETER").check("robot", Value("icub")).asString();
    if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16) {
        Vector arm_vel(16);
        for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i) {
            arm_vel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
        }
        yInfo() << log_ID << arm_vel.toString();
    }

    /* Looking for OpenGL shaders and CAD files. */
    shader_background_vert = rf.findFileByName("shader_background.vert");
    if (!fileFound(shader_background_vert)) return false;

    shader_background_frag = rf.findFileByName("shader_background.frag");
    if (!fileFound(shader_background_frag)) return false;

    shader_model_vert = rf.findFileByName("shader_model.vert");
    if (!fileFound(shader_model_vert)) return false;

    shader_model_frag = rf.findFileByName("shader_model_simple.frag");
    if (!fileFound(shader_model_frag)) return false;

    //TODO: use simplified dae instead of obj
    cad_hand["palm"] = rf.findFileByName("r_palm.obj");
    if (!fileFound(cad_hand["palm"])) return false;
    cad_hand["thumb1"] = rf.findFileByName("r_tl0.obj");
    if (!fileFound(cad_hand["thumb1"])) return false;
    cad_hand["thumb2"] = rf.findFileByName("r_tl1.obj");
    if (!fileFound(cad_hand["thumb2"])) return false;
    cad_hand["thumb3"] = rf.findFileByName("r_tl2.obj");
    if (!fileFound(cad_hand["thumb3"])) return false;
    cad_hand["thumb4"] = rf.findFileByName("r_tl3.obj");
    if (!fileFound(cad_hand["thumb4"])) return false;
    cad_hand["thumb5"] = rf.findFileByName("r_tl4.obj");
    if (!fileFound(cad_hand["thumb5"])) return false;
    cad_hand["index0"] = rf.findFileByName("r_indexbase.obj");
    if (!fileFound(cad_hand["index0"])) return false;
    cad_hand["index1"] = rf.findFileByName("r_ail0.obj");
    if (!fileFound(cad_hand["index1"])) return false;
    cad_hand["index2"] = rf.findFileByName("r_ail1.obj");
    if (!fileFound(cad_hand["index2"])) return false;
    cad_hand["index3"] = rf.findFileByName("r_ail2.obj");
    if (!fileFound(cad_hand["index3"])) return false;
    cad_hand["index4"] = rf.findFileByName("r_ail3.obj");
    if (!fileFound(cad_hand["index4"])) return false;
    cad_hand["medium0"] = rf.findFileByName("r_ml0.obj");
    if (!fileFound(cad_hand["medium0"])) return false;
    cad_hand["medium1"] = rf.findFileByName("r_ml1.obj");
    if (!fileFound(cad_hand["medium1"])) return false;
    cad_hand["medium2"] = rf.findFileByName("r_ml2.obj");
    if (!fileFound(cad_hand["medium2"])) return false;
    cad_hand["medium3"] = rf.findFileByName("r_ml3.obj");
    if (!fileFound(cad_hand["medium3"])) return false;

    /* Initializing useful pose matrices and vectors for the hand. */
    frontal_view_R.resize(3, 3);
    frontal_view_R(0,0) =  0.0;   frontal_view_R(0,1) =  0.0;   frontal_view_R(0,2) =  1.0;
    frontal_view_R(1,0) = -1.0;   frontal_view_R(1,1) =  0.0;   frontal_view_R(1,2) =  0.0;
    frontal_view_R(2,0) =  0.0;   frontal_view_R(2,1) = -1.0;   frontal_view_R(2,2) =  0.0;

    frontal_view_x.resize(3);
    frontal_view_x[0] = -0.25;
    frontal_view_x[1] = +0.00;
    frontal_view_x[2] = +0.20;

    table_view_R.resize(3, 3);
    table_view_R(0,0) = -1.0;   table_view_R(0,1) =  0.0;   table_view_R(0,2) =  0.0;
    table_view_R(1,0) =  0.0;   table_view_R(1,1) =  1.0;   table_view_R(1,2) =  0.0;
    table_view_R(2,0) =  0.0;   table_view_R(2,1) =  0.0;   table_view_R(2,2) = -1.0;

    table_view_x.resize(3);
    table_view_x[0] = -0.40;
    table_view_x[1] = +0.10;
    table_view_x[2] = +0.10;

    open_hand_joints[0] = 0;
    open_hand_joints[1] = 0;
    open_hand_joints[2] = 0;
    open_hand_joints[3] = 10;
    open_hand_joints[4] = 0;
    open_hand_joints[5] = 0;

    closed_hand_joints[0] = 80;
    closed_hand_joints[1] = 150;
    closed_hand_joints[2] = 180;
    closed_hand_joints[3] = 80;
    closed_hand_joints[4] = 10;
    closed_hand_joints[5] = 80;

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
    if (!moveFingers(open_hand_joints)) return false;

    /* Set deafult initial pose of the hand (table view). */
    if (!moveHand(table_view_R, table_view_x)) return false;

    /* Set initial finger motion point */
    radius = 0.08;
    angle_ratio = 12;
    motion_time = 10.0;
    path_time = motion_time / angle_ratio;
    itf_rightarm_cart->setTrajTime(path_time);

    /* Open a remote command port and allow the program be started */
    return setCommandPort();
}


bool SuperimposeHand::updateModule()
{
    if (start) {
        Vector motion_axis;
        Vector motion_angle;
        itf_rightarm_cart->getPose(motion_axis, motion_angle);
        Vector center(2);
        center[0] = motion_axis[0];
        center[1] = motion_axis[1] - radius;

        yInfo() << log_ID << "Starting finger motion.";
        for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angle_ratio) {
            motion_axis[0] = (center[0] - (radius * sin(alpha)));
            motion_axis[1] = (center[1] + (radius * cos(alpha)));
            yInfo() << log_ID << "Next position: [" << motion_axis.toString() << "].";
            itf_rightarm_cart->goToPose(motion_axis, motion_angle);
            Time::delay(0.7 * path_time);
        }
        yInfo() << log_ID << "Motion done.";
        if (!freerunning) start = false;
    }
    return true;
}


bool SuperimposeHand::close()
{
    yInfo() << log_ID << "Calling close functions...";

    if (superimpose_skeleton) trd_left_cam_skeleton->stop();
    if (superimpose_mesh)     trd_left_cam_cad->stop();

    delete trd_left_cam_skeleton;
    delete trd_left_cam_cad;
    
    itf_rightarm_cart->removeTipFrame();
    
    if (rightarm_cartesian_driver.isValid()) rightarm_cartesian_driver.close();
    if (rightarm_remote_driver.isValid()) rightarm_remote_driver.close();
    if (head_remote_driver.isValid()) head_remote_driver.close();
    if (gaze_driver.isValid()) gaze_driver.close();
    
    if (port_command.isOpen()) port_command.close();
    return true;
}
