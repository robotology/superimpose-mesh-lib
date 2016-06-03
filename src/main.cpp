#include <list>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;


class DumpHandSkeletonEndeffectorThread : public Thread
{
private:
    unsigned int thread_ID;
    ConstString laterality;
    ConstString camera;
    int camsel;
    
    PolyDriver &arm_remote_driver;
    PolyDriver &arm_cartesian_driver;
    PolyDriver &gaze_driver;
    
    IEncoders *itf_arm_encoders;
    int num_arm_enc;
    ICartesianControl *itf_arm_cart;
    Vector ee_x;
    Vector ee_o;
    IGazeControl *itf_head_gaze;
    Vector cam_x;
    Vector cam_o;
    
    iCubFinger finger[3];
    
    BufferedPort<ImageOf<PixelRgb>> inport_skeleton_img;
    BufferedPort<ImageOf<PixelRgb>> outport_skeleton_img;
    BufferedPort<Bottle> port_ee_pose;
    BufferedPort<Bottle> port_cam_pose;
    
public:
    DumpHandSkeletonEndeffectorThread(const unsigned int thread_ID, const ConstString &laterality, const ConstString &camera, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver) : arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver) {
        this->thread_ID = thread_ID;
        this->laterality = laterality;
        this->camera = camera;
        this->camsel = (camera == "left")? 0:1;
    }
    
    bool threadInit() {
        yInfo() << "Initializing hand skeleton drawing thread (worker:" << thread_ID << ").";
        
        yInfo() << "Setting interfaces (worker:" << thread_ID << ").";
        IControlLimits *itf_fingers_lim;
        arm_remote_driver.view(itf_fingers_lim);
        if (!itf_fingers_lim) {
            yError() << "Error getting IControlLimits interface in thread" << thread_ID << ".";
            return false;
        }
        
        arm_remote_driver.view(itf_arm_encoders);
        if (!itf_arm_encoders) {
            yError() << "Error getting IEncoders interface in thread" << thread_ID << ".";
            return false;
        }
        itf_arm_encoders->getAxes(&num_arm_enc);
        
        arm_cartesian_driver.view(itf_arm_cart);
        if (!itf_arm_cart) {
            yError() << "Error getting ICartesianControl interface in thread" << thread_ID << ".";
            return false;
        }
        
        gaze_driver.view(itf_head_gaze);
        if (!itf_head_gaze) {
            yError() << "Error getting IGazeControl interface in thread" << thread_ID << ".";
            return false;
        }
        yInfo() << "Interfaces set (worker:" << thread_ID << ")!";
        
        yInfo() << "Setting joint bounds for the fingers (worker:" << thread_ID << ").";
        finger[0] = iCubFinger(laterality+"_thumb");
        finger[1] = iCubFinger(laterality+"_index");
        finger[2] = iCubFinger(laterality+"_middle");
        
        std::deque<IControlLimits*> temp_lim;
        temp_lim.push_front(itf_fingers_lim);
        for (int i = 0; i < 3; ++i) {
            if (!finger[i].alignJointsBounds(temp_lim)) {
                yError() << "Cannot set joint bound for finger" << i << "(worker:" << thread_ID << ").";
                return false;
            }
        }
        yInfo() << "Joint bound for finger set (worker:" << thread_ID << ")!";
        
        yInfo() << "Opening ports for skeleton images (worker:" << thread_ID << ").";
        if (!inport_skeleton_img.open("/movefinger/img_skeleton_"+camera+":i")) {
            yError() << "Cannot open skeleton image input port for "+camera+" camera" << "in thread" << thread_ID << ".";
            return false;
        }
        if (!outport_skeleton_img.open("/movefinger/img_skeleton_"+camera+":o")) {
            yError() << "Cannot open skeleton image output port for "+camera+" camera" << "in thread" << thread_ID << ".";
            return false;
        }
        yInfo() << "Skeleton image ports succesfully opened (worker:" << thread_ID << ")!";
        
        yInfo() << "Initializing end effector pose dumping thread (worker:" << thread_ID << ").";
        if (!port_ee_pose.open("/movefinger/endeffector_pose:o")) {
            yError() << "Cannot open /movefinger/endeffector_pose:o port (worker:" << thread_ID << ").";
            return false;
        }
        
        ee_x.resize(3);
        ee_o.resize(4);
        yInfo() << "End effector port succesfully opened (worker:" << thread_ID << ").";
        
        yInfo() << "Initializing"+camera+"camera pose dumping thread (worker:" << thread_ID << ").";
        if (!port_cam_pose.open("/movefinger/"+camera+"_camera_pose:o")) {
            yError() << "Cannot open /movefinger/"+camera+"_camera_pose:o port (worker:" << thread_ID << ").";
            return false;
        }
        
        cam_x.resize(3);
        cam_o.resize(4);
        yInfo() << "Port for "+camera+" camera succesfully opened (worker:" << thread_ID << ").";
        
        Bottle bottle;
        itf_head_gaze->getInfo(bottle);
        yInfo() << "Camera Info:"<< bottle.toString();
        
        yInfo() << "Initialization completed for worker" << thread_ID << ".";
        
        return true;
    }
    
    void run() {
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
                    
                    for (unsigned int i = 0; i < finger[fng].getN(); ++i) {
                        Vector current_joint_pixel;
                        itf_head_gaze->get2DPixel(camsel, Ha*(finger[fng].getH(i, true).getCol(3)), current_joint_pixel);
                        
                        current_joint_point.push_front(cv::Point(static_cast<int>(current_joint_pixel[0]), static_cast<int>(current_joint_pixel[1])));
                        cv::circle(img, current_joint_point.front(), 3, cv::Scalar(0, 0, 255), 4);
                        
                        if (i > 0) {
                            cv::line(img, current_joint_point.front(), current_joint_point.back(), cv::Scalar(255, 255, 255), 2);
                            current_joint_point.pop_back();
                        }
                        else {
                            cv::line(img, endeffector_point, current_joint_point.front(), cv::Scalar(255, 0, 0), 2);
                        }
                    }
                }
                
                Bottle &eePoseBottle = port_ee_pose.prepare();
                eePoseBottle.clear();
                eePoseBottle.addString(ee_x.toString() + "    " + ee_o.toString());
                
                Bottle &camPoseBottle = port_cam_pose.prepare();
                camPoseBottle.clear();
                camPoseBottle.addString(cam_x.toString() + "    " + cam_o.toString());
                
                outport_skeleton_img.write();
                port_ee_pose.write();
                port_cam_pose.write();
            }
        }
    }
    
    void threadRelease() {
        yInfo() << "Deallocating resource of hand skeleton drawing thread for worker " << thread_ID << ".";
        
        if (!inport_skeleton_img.isClosed()) inport_skeleton_img.close();
        if (!outport_skeleton_img.isClosed()) outport_skeleton_img.close();
        if (!port_ee_pose.isClosed()) port_ee_pose.close();
        if (!port_cam_pose.isClosed()) port_cam_pose.close();
        
        yInfo() << "Deallocation completed for worker " << thread_ID << ".";
    }
};


class MoveFinger : public RFModule
{
private:
    ConstString robot;
    bool usegaze;
    bool usetorsoDOF;
    bool start;
    bool viewhand;
    bool freerunning;
    bool dumping_data;

    PolyDriver rightarm_remote_driver;
    IEncoders *itf_rightarm_enc;
    IPositionControl2 *itf_rightarm_pos;
    int num_rightarm_joint;
    int num_rightarm_enc;

    PolyDriver rightarm_cartesian_driver;
    ICartesianControl *itf_rightarm_cart;

    PolyDriver head_remote_driver;
    IPositionControl2 *itf_head_pos;
    int num_head_joint;

    PolyDriver gaze_driver;
    IGazeControl *itf_head_gaze;

    DumpHandSkeletonEndeffectorThread *thread_dump_hand_ee;

    Port port_command;

    Matrix frontal_view_R;
    Vector frontal_view_x;
    
    Matrix table_view_R;
    Vector table_view_x;
    
    double open_hand_joints[6];
    double closed_hand_joints[6];
    
    double radius;
    int angle_ratio;
    double motion_time;
    double path_time;

    bool setRightArmRemoteControlboard()
    {
        Property rightarm_remote_options;
        rightarm_remote_options.put("device", "remote_controlboard");
        rightarm_remote_options.put("local", "/movefinger/control_right_arm");
        rightarm_remote_options.put("remote", "/"+robot+"/right_arm");

        rightarm_remote_driver.open(rightarm_remote_options);
        if (rightarm_remote_driver.isValid()) {
            yInfo() << "Right arm remote_controlboard succefully opened.";

            rightarm_remote_driver.view(itf_rightarm_enc);
            if (!itf_rightarm_enc) {
                yError() << "Error getting right arm IEncoders interface.\n";
                return false;
            }
            num_rightarm_enc = 0;
            itf_rightarm_enc->getAxes(&num_rightarm_enc);
            yInfo() << "Right arm encorders succefully read.";

            rightarm_remote_driver.view(itf_rightarm_pos);
            if (!itf_rightarm_pos) {
                yError() << "Error getting right arm IPositionControl2 interface.\n";
                return false;
            }
            yInfo() << "Right arm positions succefully read.";
        } else {
            yError() << "Error opening right arm remote_controlboard device.\n";
            return false;
        }

        num_rightarm_joint = 0;
        itf_rightarm_pos->getAxes(&num_rightarm_joint);
        yInfo() << "Total number of right arm joints: " << num_rightarm_joint << ".";
        Vector tmp(static_cast<size_t>(num_rightarm_joint));
        for (int i = 0; i < num_rightarm_joint; ++i) {
            tmp[i] = 10.0;
        }
        if (!itf_rightarm_pos->setRefAccelerations(tmp.data()))
        {
            yError() << "Error setting right arm joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < num_rightarm_joint; ++i) {
            tmp[i] = 15.0;
            if (!itf_rightarm_pos->setRefSpeed(i, tmp[i]))
            {
                yError() << "Error setting right arm joint speeds.\n";
                return false;
            }
        }
        yInfo() << "Right arm joint speeds and accelerations succesfully set.";

        return true;
    }

    bool setRightArmCartesianController()
    {
        Property rightarm_cartesian_options;
        rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
        rightarm_cartesian_options.put("local", "/movefinger/cart_right_arm");
        rightarm_cartesian_options.put("remote", "/"+robot+"/cartesianController/right_arm");

        rightarm_cartesian_driver.open(rightarm_cartesian_options);
        if (rightarm_cartesian_driver.isValid()) {
            rightarm_cartesian_driver.view(itf_rightarm_cart);
            if (!itf_rightarm_cart) {
                yError() << "Error getting ICartesianControl interface.\n";
                return false;
            }
            yInfo() << "cartesiancontrollerclient succefully opened.";
        } else {
            yError() << "Error opening cartesiancontrollerclient device.\n";
            return false;
        }

        return true;
    }

    bool setHeadRemoteControlboard()
    {
        Property head_option;
        head_option.put("device", "remote_controlboard");
        head_option.put("local", "/movefinger/control_head");
        head_option.put("remote", "/"+robot+"/head");

        head_remote_driver.open(head_option);
        if (head_remote_driver.isValid()) {
            yInfo() << "Head remote_controlboard succefully opened.";

            head_remote_driver.view(itf_head_pos);
            if (!itf_head_pos) {
                yError() << "Error getting head IPositionControl interface.\n";
                return false;
            }
            yInfo() << "Head positions succefully read.";
        } else {
            yError() << "Error opening head remote_controlboard device.";
            return false;
        }
        
        num_head_joint = 0;
        itf_head_pos->getAxes(&num_head_joint);
        yInfo() << "Total number of head joints: " << num_head_joint << ".";
        Vector tmp(static_cast<size_t>(num_head_joint));
        for (int i = 0; i < num_head_joint; ++i) {
            tmp[i] = 10.0;
        }
        if (!itf_head_pos->setRefAccelerations(tmp.data()))
        {
            yError() << "Error setting head joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < num_head_joint; ++i) {
            tmp[i] = 15.0;
            if (!itf_head_pos->setRefSpeed(i, tmp[i]))
            {
                yError() << "Error setting head joint speeds.\n";
                return false;
            }
        }
        yInfo() << "Head joint speeds and accelerations set.";

        return true;
    }

    bool setGazeController()
    {
        Property gaze_option;
        gaze_option.put("device", "gazecontrollerclient");
        gaze_option.put("local", "/movefinger/gaze");
        gaze_option.put("remote", "/iKinGazeCtrl");

        gaze_driver.open(gaze_option);
        if (gaze_driver.isValid()) {
            gaze_driver.view(itf_head_gaze);
            if (!itf_head_gaze) {
                std::cerr << "Error getting IGazeControl interface.\n";
                return false;
            }
        } else {
            std::cerr << "Gaze control device not available.\n";
            return false;
        }

        return true;
    }

    bool setTorsoDOF()
    {
        Vector curDOF;
        itf_rightarm_cart->getDOF(curDOF);
        yInfo() << "Old DOF: [" + curDOF.toString(0) + "].";
        yInfo() << "Setting iCub to use the DOF from the torso.";
        Vector newDOF(curDOF);
        newDOF[0] = 1;
        newDOF[1] = 2;
        newDOF[2] = 1;
        if (!itf_rightarm_cart->setDOF(newDOF, curDOF)) {
            yError() << "Cannot use torso DOF.";
            return false;
        }
        yInfo() << "Setting the DOF done.";
        yInfo() << "New DOF: [" + curDOF.toString(0) + "]";

        return true;
    }

    bool setCommandPort()
    {
        yInfo() << "Opening command port.";
        if (!port_command.open("/movefinger/rpc")) {
            yError() << "Cannot open the command port.";
            return false;
        }
        if (!attach(port_command)) {
            yError() << "Cannot attach the command port.";
            return false;
        }
        yInfo() << "Command port succesfully opened and attached. Ready to start and recieve commands.";

        return true;
    }
    
    bool moveFingers(const double (&joint)[6])
    {
        /* Close iCub hand. */
        yInfo() << "Closing fingers.";
        Vector rightarm_encoder(static_cast<size_t>(num_rightarm_enc));
        itf_rightarm_enc->getEncoders(rightarm_encoder.data());
        std::list<std::pair<unsigned int, double>> joint_pos_map = {{13, joint[0]},
                                                                    {14, joint[1]},
                                                                    {15, joint[2]},
                                                                    { 8, joint[3]},
                                                                    { 9, joint[4]},
                                                                    {10, joint[5]}};
        for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map) {
            yInfo() << "Moving joint" << map->first << "to the position" << map->second << ".";
            if (std::abs(rightarm_encoder[map->first] - map->second) > 5.0) {
                rightarm_encoder[map->first] = map->second;
                itf_rightarm_pos->positionMove(rightarm_encoder.data());
                Time::delay(2.0);
            }
        }
        yInfo() << "Fingers succesfully closed.";
        
        return true;
    }
    
    bool moveHand(const Matrix &R, const Vector &init_x)
    {
        /* Setting hand pose */
        yInfo() << "Moving hand to the initial position.";
        
        Vector init_o(dcm2axis(R));
        
        itf_rightarm_cart->goToPoseSync(init_x, init_o);
        itf_rightarm_cart->waitMotionDone(0.1, 6.0);
        
        yInfo() << "The hand is in position.";
        
        /* Set initial fixation point */
        if (usegaze) {
            Vector tmp;
            itf_head_gaze->getFixationPoint(tmp);
            Vector init_fixation(init_x);
            init_fixation[0] -= 0.05;
            init_fixation[1] -= 0.05;
            if (norm(tmp - init_fixation) > 0.10) {
                yInfo() << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
                itf_head_gaze->lookAtFixationPoint(init_fixation);
                itf_head_gaze->waitMotionDone(0.1, 6.0);
            }
            yInfo() << "Gaze motion done.";
        }
        
        return true;
    }

public:
    double getPeriod() { return 0.0; }

    bool configure(ResourceFinder &rf)
    {
        /* Setting default parameters. */
        start = false;
        viewhand = false;
        freerunning = false;
        dumping_data = false;

        /* Parsing parameters from config file. */
        robot = rf.findGroup("PARAMETER").check("robot", Value("icub")).asString();
        usegaze = rf.findGroup("PARAMETER").check("usegaze", Value(true)).asBool();
        usetorsoDOF = rf.findGroup("PARAMETER").check("usetorsodof", Value(true)).asBool();
        if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16) {
            Vector arm_vel(16);
            for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i) {
                arm_vel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
            }
            yInfo() << arm_vel.toString();
        }
        
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
        if (usegaze && !setGazeController()) return false;

        /* Enable torso DOF. */
        if (usetorsoDOF && !setTorsoDOF()) return false;

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

    bool respond(const Bottle& command, Bottle& reply)
    {
        yInfo() << "Got something: " << command.toString();

        if (command.get(0).asString() == "start") {
            
            if (viewhand) {
                Bottle("Can't move hand in this settings! Use tablehand command before using start command again.");
            } else {
                start = true;
                
                if (command.get(1).asString() == "freerun") {
                    freerunning = true;
                    reply = Bottle("Starting freerunning motion mode.");
                } else {
                    reply = Bottle("Starting single finger motion.");
                }
            }
        }
        else if (command.get(0).asString() == "stop") {

            start = false;

            if (freerunning) freerunning = false;

            reply = Bottle("Stopping finger motion.");

        }
        else if (command.get(0).asString() == "tablehand") {
            
            if (!viewhand) {
                reply = Bottle("Already in tablehand settings!");
            } else {
                viewhand = false;
                
                reply = Bottle("Tablehand enabled, iCub can move the hand in this settings.");
                
                moveHand(table_view_R, table_view_x);
            }
            
        }
        else if (command.get(0).asString() == "viewhand") {

            if (start) {
                reply = Bottle("Can't move hand while moving it!");
            } else {
                viewhand = true;
                
                reply = Bottle("Viewhand enabled, iCub can't move hand in this settings.");
                
                moveHand(frontal_view_R, frontal_view_x);
            }

        }
        else if (command.get(0).asString() == "fingers") {
            
            if (command.get(1).asString() == "open") {
                moveFingers(open_hand_joints);
                reply = Bottle("Hand opened!");
            }
            else if (command.get(1).asString() == "close") {
                moveFingers(closed_hand_joints);
                reply = Bottle("Hand closed (...but the index finger)!");
            }
            else {
                reply = Bottle("Option not available for fingers settings (available: open, close).");
            }
            
        }
        else if (command.get(0).asString() == "dumpdata"){
            
            if (!dumping_data && command.get(1).asString() == "on") {
                thread_dump_hand_ee = new DumpHandSkeletonEndeffectorThread(1, "right", "left", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver);
                
                if (thread_dump_hand_ee != NULL) {
                    reply = Bottle("Starting hand skeleton and end effector data dumping thread.");
                    thread_dump_hand_ee->start();
                    
                    dumping_data = true;
                }
                else {
                    reply = Bottle("Could not initialize hand skeleton and end effector data dumping thread.");
                }
                
            }
            else if (dumping_data && command.get(1).asString() == "off") {
                reply = Bottle("Stopping end effector data dumping thread.");
                
                thread_dump_hand_ee->stop();
                dumping_data = false;
                delete thread_dump_hand_ee;
            }
            else {
                reply = Bottle("Option not available for end effector dumping thread operations (available: on, off).");
            }
            
        }
        else if (command.get(0).asString() == "quit") {

            reply = Bottle("Quitting...");
            return false;

        }
        else {
            reply = Bottle("Not a valid command.");
        }

        return true;
    }

    bool updateModule()
    {
        if (start) {
            Vector motion_axis;
            Vector motion_angle;
            itf_rightarm_cart->getPose(motion_axis, motion_angle);
            Vector center(2);
            center[0] = motion_axis[0];
            center[1] = motion_axis[1] - radius;
            
            yInfo() << "Starting finger motion.";
            for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angle_ratio) {
                motion_axis[0] = (center[0] - (radius * sin(alpha)));
                motion_axis[1] = (center[1] + (radius * cos(alpha)));
                yInfo() << "Next position: [" << motion_axis.toString() << "].";
                itf_rightarm_cart->goToPose(motion_axis, motion_angle);
                Time::delay(0.7 * path_time);
            }
            yInfo() << "Motion done.";
            if (!freerunning) start = false;
        }
        return true;
    }

    bool interruptModule()
    {
        yInfo() << "Interrupting the module...\nStopping threads...";

        if (dumping_data) {
            thread_dump_hand_ee->stop();
            delete thread_dump_hand_ee;
        }

        return true;
    }

    bool close()
    {
        yInfo() << "Calling close functions...";

        itf_rightarm_cart->removeTipFrame();

        if (rightarm_cartesian_driver.isValid()) rightarm_cartesian_driver.close();
        if (rightarm_remote_driver.isValid()) rightarm_remote_driver.close();
        if (head_remote_driver.isValid()) head_remote_driver.close();
        if (gaze_driver.isValid()) gaze_driver.close();
        if (port_command.isOpen()) port_command.close();

        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP seems unavailable.";
        return -1;
    }

    ResourceFinder rf;
    MoveFinger module;
    yInfo() << "Configuring and starting module...";

    rf.setVerbose(true);
    rf.setDefaultConfigFile("movefinger_config.ini");
    rf.setDefaultContext("movefinger");
    rf.configure(argc, argv);

    module.runModule(rf);

    yInfo() << "Main returning.";
    yInfo() << "Application closed.";
    return 0;
}
