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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

// TODO: in realta' fornisce la posizione dell'end effector. Cambiarlo o sdoppiarlo con punto dito e end effector.
class WriteEndEffectorPoseThread : public Thread
{
private:
    unsigned int threadID;
    ICartesianControl *itf_arm_cart;

    Vector tip_x;
    Vector tip_o;
    BufferedPort<Bottle> port_tip_pose;

public:

    WriteEndEffectorPoseThread(unsigned int threadID, ICartesianControl *&itf_arm_cart) {
        this->threadID = threadID;
        this->itf_arm_cart = itf_arm_cart;
    }

    bool threadInit() {
        yInfo() << "Initializing end effector pose dumping thread (worker:" << threadID << ").";

        /* Open buffered port for sending the current tip pose */
        if (!port_tip_pose.open("/movefinger/endeffector_pose:o")) {
            yError() << "Cannot open /movefinger/endeffector_pose:o port.";
            return false;
        }

        tip_x.resize(3);
        tip_o.resize(4);

        yInfo() << "Initialization completed for worker" << threadID << ".";

        return true;
    }

    void run() {
        while(!isStopping()) {

            Bottle &tipPoseBottle = port_tip_pose.prepare();

            itf_arm_cart->getPose(tip_x, tip_o);

            tipPoseBottle.clear();
            tipPoseBottle.addString(tip_x.toString() + " " + tip_o.toString());

            port_tip_pose.write();

        }
    }

    void threadRelease() {
        yInfo() << "Deallocating resource of end effector pose dumping thread (worker:" << threadID << ").";

        if (!port_tip_pose.isClosed()) port_tip_pose.close();

        yInfo() << "Deallocation completed for worker:" << threadID << ".";
    }
};

class WriteFingerTipPoseThread : public Thread
{
private:
    unsigned int threadID;
    ConstString finger;
    IEncoders *itf_arm_enc;
    ICartesianControl *itf_arm_cart;

    int num_rightarm_enc;
    Vector encs;
    Vector xa;
    Vector oa;
    Matrix Ha;
    Vector tip_chain_joint;
    iCubFinger tip_finger;
    Matrix tip_frame;
    Vector tip_x;
    Vector tip_o;
    BufferedPort<Bottle> port_tip_pose;

public:

    WriteFingerTipPoseThread(unsigned int threadID, ConstString finger, IEncoders *&itf_arm_enc, ICartesianControl *itf_arm_cart) {
        this->threadID = threadID;
        this->finger = finger;
        this->itf_arm_enc = itf_arm_enc;
        this->itf_arm_cart = itf_arm_cart;
    }

    bool threadInit() {
        yInfo() << "Initializing tip pose dumping thread (worker:" << threadID << ").";

        num_rightarm_enc = 0;
        itf_arm_enc->getAxes(&num_rightarm_enc);

        /* Open buffered port for sending the current tip pose */
        if (!port_tip_pose.open("/movefinger/finger_tip_pose:o")) {
            yError() << "Cannot open /movefinger/finger_tip_pose:o port.";
            return false;
        }

        encs.resize(static_cast<size_t>(num_rightarm_enc));
        tip_finger = iCubFinger(finger);
        tip_x.resize(4);
        tip_o.resize(4);

        yInfo() << "Initialization completed for worker" << threadID << ".";

        return true;
    }

    void run() {
        while(!isStopping()) {

            Bottle &tipPoseBottle = port_tip_pose.prepare();

            if (!itf_arm_cart->getPose(xa, oa)) {
                yError() << "Thread" << threadID << ": cannot read end effector position.";
            }
            else {
                Ha = axis2dcm(oa);
                xa.push_back(1.0);
                Ha.setCol(3, xa);

                if (!itf_arm_enc->getEncoders(encs.data())) {
                    yError() << "Thread" << threadID << ": cannot read from encoders.";
                }
                else {
                    if (!tip_finger.getChainJoints(encs, tip_chain_joint)) {
                        yError() << "Thread" << threadID << ": cannot get joint information to the" << finger << "tip.";
                    }
                    else {
                        tip_finger.setAng(CTRL_DEG2RAD * tip_chain_joint);

                        tip_frame = Ha * tip_finger.getH((M_PI / 180.0) * tip_chain_joint);
                        tip_x = tip_frame.getCol(3);
                        tip_o = dcm2axis(tip_frame);

                        tipPoseBottle.clear();
                        tipPoseBottle.addString(tip_x.subVector(0, 2).toString() + " " + tip_o.toString());

                        port_tip_pose.write();
                    }
                }
            }
        }
    }

    void threadRelease() {
        yInfo() << "Deallocating resource of tip pose dumping thread (worker:" << threadID << ").";

        if (!port_tip_pose.isClosed()) port_tip_pose.close();

        yInfo() << "Deallocation completed for worker:" << threadID << ".";
    }
};

class DrawHandSkeletonThread : public Thread
{
private:
    unsigned int thread_ID;
    ConstString laterality;
    PolyDriver &arm_remote_driver;
    PolyDriver &arm_cartesian_driver;
    PolyDriver &gaze_driver;

    IControlLimits *itf_fingers_lim;
    IEncoders *itf_arm_encoders;
    int num_arm_enc;
    ICartesianControl *itf_arm_cart;
    IGazeControl *itf_head_gaze;
    iCubFinger finger[3];
    BufferedPort<ImageOf<PixelRgb>> inport_skeleton_img_left;
    BufferedPort<ImageOf<PixelRgb>> outport_skeleton_img_left;
    BufferedPort<ImageOf<PixelRgb>> inport_skeleton_img_right;
    BufferedPort<ImageOf<PixelRgb>> outport_skeleton_img_right;

    void drawHandSkeleton(ConstString camera, BufferedPort<ImageOf<PixelRgb>> &inport, BufferedPort<ImageOf<PixelRgb>> &outport) {
        ImageOf<PixelRgb> *imgin = inport.read(false);
        if (imgin != NULL) {

            int camSel = (camera == "left")? 0:1;

            ImageOf<PixelRgb> &imgout = outport.prepare();
            imgout = *imgin;

            cv::Mat img = cv::cvarrToMat(imgout.getIplImage());

            Vector xa;
            Vector oa;
            itf_arm_cart->getPose(xa, oa);
            Matrix Ha = axis2dcm(oa);
            xa.push_back(1.0);
            Ha.setCol(3, xa);

            Vector endeffector_pixel;
            itf_head_gaze->get2DPixel(camSel, xa, endeffector_pixel);

            cv::Point endeffector_point((int) endeffector_pixel[0], (int) endeffector_pixel[1]);
            cv::circle(img, endeffector_point, 4, cv::Scalar(0, 255, 0), 4);

            Vector encs(static_cast<size_t>(num_arm_enc));
            Vector chainjoints;
            itf_arm_encoders->getEncoders(encs.data());
            for (unsigned int i = 0; i < 3; ++i) {
                finger[i].getChainJoints(encs, chainjoints);
                finger[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            for (unsigned int fng = 0; fng < 3; ++fng) {
                deque<cv::Point> current_joint_point;

                for (unsigned int i = 0; i < finger[fng].getN(); ++i) {
                    Vector current_joint_pixel;
                    itf_head_gaze->get2DPixel(camSel, Ha*(finger[fng].getH(i, true).getCol(3)), current_joint_pixel);

                    current_joint_point.push_front(cv::Point((int) current_joint_pixel[0], (int) current_joint_pixel[1]));
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

            outport.write();
        }
    }

public:
    DrawHandSkeletonThread(unsigned int thread_ID, ConstString laterality, PolyDriver &arm_remote_driver, PolyDriver &arm_cartesian_driver, PolyDriver &gaze_driver) : arm_remote_driver(arm_remote_driver), arm_cartesian_driver(arm_cartesian_driver), gaze_driver(gaze_driver){
        this->thread_ID = thread_ID;
        this->laterality = laterality;
    }

    bool threadInit() {
        yInfo() << "Initializing hand skeleton drawing thread (worker:" << thread_ID << ").";

        yInfo() << "Setting interfaces.";
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

        yInfo() << "Setting joint bounds for the fingers.";

        finger[0] = iCubFinger(laterality+"_thumb");
        finger[1] = iCubFinger(laterality+"_index");
        finger[2] = iCubFinger(laterality+"_middle");

        deque<IControlLimits*> temp_lim;
        temp_lim.push_front(itf_fingers_lim);
        for (int i = 0; i < 3; ++i) {
            if (!finger[i].alignJointsBounds(temp_lim)) {
                yError() << "Cannot set joint bound for finger" << i << "in thread" << thread_ID << ".";
                return false;
            }
        }
        yInfo() << "Joint bound for finger set!";

        yInfo() << "Opening ports for skeleton images.";
        if (!inport_skeleton_img_left.open("/movefinger/img_skeleton_left:i")) {
            yError() << "Cannot open skeleton image input port for left camera" << "in thread" << thread_ID << ".";
            return false;
        }
        if (!outport_skeleton_img_left.open("/movefinger/img_skeleton_left:o")) {
            yError() << "Cannot open skeleton image output port for left camera" << "in thread" << thread_ID << ".";
            return false;
        }

        if (!inport_skeleton_img_right.open("/movefinger/img_skeleton_right:i")) {
            yError() << "Cannot open skeleton image input port for right camera" << "in thread" << thread_ID << ".";
            return false;
        }
        if (!outport_skeleton_img_right.open("/movefinger/img_skeleton_right:o")) {
            yError() << "Cannot open skeleton image output port for right camera" << "in thread" << thread_ID << ".";
            return false;
        }

        yInfo() << "Skeleton image ports succesfully opened.";

        yInfo() << "Initialization completed for worker" << thread_ID << ".";

        return true;
    }

    void run() {
        while (!isStopping()) {
            drawHandSkeleton("left", inport_skeleton_img_left, outport_skeleton_img_left);
            drawHandSkeleton("right", inport_skeleton_img_right, outport_skeleton_img_right);
        }
    }

    void threadRelease() {
        yInfo() << "Deallocating resource of hand skeleton drawing thread (worker:" << thread_ID << ").";

        if (!outport_skeleton_img_left.isClosed()) outport_skeleton_img_left.close();
        if (!inport_skeleton_img_left.isClosed()) inport_skeleton_img_left.close();
        if (!outport_skeleton_img_right.isClosed()) outport_skeleton_img_right.close();
        if (!inport_skeleton_img_right.isClosed()) inport_skeleton_img_right.close();

        yInfo() << "Deallocation completed for worker:" << thread_ID << ".";
    }
};

// TODO: scindere movefinger in un thread + un programma principale
class MoveFinger : public RFModule
{
private:
    ConstString robot;
    bool usegaze;
    bool usetorsoDOF;
    bool moveendeffectortotip;
    Vector arm_vel;

    Property rightarm_remote_options;
    PolyDriver rightarm_remote_driver;
    IEncoders *itf_rightarm_enc;
    IPositionControl2 *itf_rightarm_pos;
    int num_rightarm_joint;
    int num_rightarm_enc;

    Property rightarm_cartesian_options;
    PolyDriver rightarm_cartesian_driver;
    ICartesianControl *itf_rightarm_cart;

    Property head_option;
    PolyDriver head_remote_driver;
    IPositionControl2 *itf_head_pos;
    int num_head_joint;

    Property gaze_option;
    PolyDriver gaze_driver;
    IGazeControl *itf_head_gaze;

    iCubFinger *tip_finger;
    Vector tip_chain_joint;
    Matrix tip_frame;
    Vector tip_x;
    Vector tip_o;

    Vector rightarm_encoder;
    list<pair<unsigned int, double>> joint_pos_map;
    Matrix R;
    Vector init_x;
    Vector init_o;
    Vector init_fixation;

    WriteEndEffectorPoseThread *thread_wtp;
    WriteFingerTipPoseThread *thread_wftp;
    DrawHandSkeletonThread *thread_dhs_right;

    Port port_command;

    bool verbose;
    bool start;
    bool freerunning;
    bool dumpdata_ee;
    bool dumpdata_finger;
    bool showrightskeleton;

    Vector tip_motion_axis;
    Vector tip_motion_angle;
    Vector center;
    double radius;
    int angle_ratio;
    double motion_time;
    double path_time;

    bool setRightArmRemoteControlboard()
    {
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

    bool setTipFrame()
    {
        yInfo() << "Moving the end effector reference frame to the right index tip.";

        Vector encs(static_cast<size_t>(num_rightarm_enc));
        if (!itf_rightarm_enc->getEncoders(encs.data())) {
            yError() << "Cannot read from encoders.";
            return false;
        }

        tip_finger = new iCubFinger("right_index");
        if (!tip_finger->getChainJoints(encs, tip_chain_joint)) {
            yError() << "Cannot get joint information to the finger tip.";
            return false;
        }

        tip_frame = tip_finger->getH((M_PI / 180.0) * tip_chain_joint);
        tip_x = tip_frame.getCol(3);
        tip_o = dcm2axis(tip_frame);
        if (!itf_rightarm_cart->attachTipFrame(tip_x, tip_o)) {
            yError() << "Cannot attach reference frame to finger tip.";
            return false;
        }
        yInfo() << "The end effector reference frame is now onto the right index tip.";

        return true;
    }

    bool setCommandPort()
    {
        yInfo() << "Opening command port.";
        if (!port_command.open("/movefinger/command:i")) {
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

public:
    double getPeriod() { return 0.0; }

    bool configure(ResourceFinder &rf)
    {
        /* Setting default parameters */
        verbose = false;
        start = false;
        freerunning = false;
        dumpdata_ee = false;
        dumpdata_finger = false;

        /* Parsing parameters from config file */
        robot = rf.findGroup("PARAMETER").check("robot", Value("icub")).asString();
        usegaze = rf.findGroup("PARAMETER").check("usegaze", Value(false)).asBool();
        usetorsoDOF = rf.findGroup("PARAMETER").check("usetorsodof", Value(true)).asBool();
        moveendeffectortotip = rf.findGroup("PARAMETER").check("moveendeffectortotip", Value(false)).asBool();
        if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16) {
            arm_vel.resize(16);
            for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i) {
                arm_vel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
            }
        }
        yInfo() << arm_vel.toString();

        /* Right arm control board */
        if (!setRightArmRemoteControlboard()) return false;

        /* Right arm cartesian controler */
        if (!setRightArmCartesianController()) return false;

        /* Head control board */
        if (!setHeadRemoteControlboard()) return false;

        /* Gaze control */
        if (usegaze && !setGazeController()) return false;

        /* Enable torso DOF */
        if (usetorsoDOF && !setTorsoDOF()) return false;

        /* Move reference framework to the right index tip */
        if (moveendeffectortotip && !setTipFrame()) return false;

        /* Set hand configuration */
        yInfo() << "Closing fingers.";
        rightarm_encoder.resize(static_cast<size_t>(num_rightarm_enc));
        itf_rightarm_enc->getEncoders(rightarm_encoder.data());
        joint_pos_map = {{13, 80},
                         {14, 150},
                         {15, 180},
                         {8, 80},
                         {9, 10},
                         {10, 80}};
        for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map) {
            yInfo() << "Moving joint" << map->first << "to the position" << map->second << ".";
            if (abs(rightarm_encoder[map->first] - map->second) > 5.0) {
                rightarm_encoder[map->first] = map->second;
                itf_rightarm_pos->positionMove(rightarm_encoder.data());
            }
        }
        yInfo() << "Fingers succesfully closed.";

        // TODO: sistemare la posizione della mano a seconda dell'end effector scelto: palmo o dito
//        /* Set initial hand pose */
//        yInfo() << "Moving hand to the initial position.";
//        R.resize(3, 3);
//        R(0,0) = -1.0;   R(0,1) =  0.0;   R(0,2) =  0.0;
//        R(1,0) =  0.0;   R(1,1) =  1.0;   R(1,2) =  0.0;
//        R(2,0) =  0.0;   R(2,1) =  0.0;   R(2,2) = -1.0;
//        init_o = dcm2axis(R);
//        init_x.resize(3);
//        init_x[0] = -0.35;
//        init_x[1] = +0.20;
//        init_x[2] = +0.20;
//        itf_rightarm_cart->goToPoseSync(init_x, init_o);
//        itf_rightarm_cart->waitMotionDone(0.1, 6.0);
//        yInfo() << "The hand is in position.";

        /* Set initial hand pose */
        yInfo() << "Moving hand to the initial position.";
        R.resize(3, 3);
        R(0,0) =  0.0;   R(0,1) = -1.0;   R(0,2) =  0.0;
        R(1,0) = -1.0;   R(1,1) =  0.0;   R(1,2) =  0.0;
        R(2,0) =  0.0;   R(2,1) =  0.0;   R(2,2) = -1.0;
        init_o = dcm2axis(R);
        init_x.resize(3);
        init_x[0] = -0.25;
        init_x[1] = +0.00;
        init_x[2] = +0.20;
        itf_rightarm_cart->goToPoseSync(init_x, init_o);
        itf_rightarm_cart->waitMotionDone(0.1, 6.0);
        yInfo() << "The hand is in position.";

        /* Set initial fixation point */
        if (usegaze) {
            Vector tmp;
            itf_head_gaze->getFixationPoint(tmp);
            init_fixation = init_x;
            init_fixation[0] -= 0.05;
            init_fixation[1] -= 0.05;
            if (norm(tmp - init_fixation) > 0.10) {
                yInfo() << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
                itf_head_gaze->lookAtFixationPoint(init_fixation);
                itf_head_gaze->waitMotionDone(0.1, 6.0);
            }
            yInfo() << "Gaze motion done.";
        }

        /* Set initial finger motion point */
        tip_motion_axis.resize(3);
        tip_motion_axis[2] = init_x[2];
        tip_motion_angle = init_o;
        radius = 0.10;
        center.resize(2);
        center[0] = init_x[0];
        center[1] = init_x[1] - radius;
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

            start = true;

            if (command.get(1).asString() == "freerun") {
                freerunning = true;
                reply = Bottle("Starting freerunning motion mode.");
            } else {
                reply = Bottle("Starting single finger motion.");
            }

        }
        else if (command.get(0).asString() == "stop") {

            start = false;

            if (freerunning) freerunning = false;
            reply = Bottle("Stopping finger motion.");

        }
        else if (command.get(0).asString() == "dumpdata"){

            if (!dumpdata_ee && command.get(1).asString() == "ee") {

                if (command.get(2).asString() == "on") {
                    thread_wtp = new WriteEndEffectorPoseThread(1, itf_rightarm_cart);

                    if (thread_wtp != NULL) {
                        reply = Bottle("Starting end effector data dumping thread.");
                        thread_wtp->start();

                        dumpdata_ee = true;
                    }
                    else {
                        reply = Bottle("Could not initialize end effector dumping thread.");
                    }

                }
                else if (command.get(2).asString() == "off") {

                    reply = Bottle("Stopping end effector data dumping thread.");

                    thread_wtp->stop();
                    delete thread_wtp;

                    dumpdata_ee = false;
                }
                else {
                    reply = Bottle("Option not available for end effector dumping thread operations (available: on, off).");
                }

            }
            else if (!dumpdata_finger && command.get(1).asString() == "finger") {

                if (command.get(2).asString() == "on") {

                    thread_wftp = new WriteFingerTipPoseThread(11, "right_index", itf_rightarm_enc, itf_rightarm_cart);

                    if (thread_wftp != NULL) {
                        reply = Bottle("Starting tip pose data dumping thread.");
                        thread_wftp->start();

                        dumpdata_finger = true;
                    }
                    else {
                        reply = Bottle("Could not initialize tip pose dumping thread.");
                    }

                }
                else if (command.get(2).asString() == "off") {

                    reply = Bottle("Stopping tip pose data dumping thread.");

                    thread_wftp->stop();
                    delete thread_wftp;

                    dumpdata_finger = false;
                }

            }
            else {
                reply = Bottle("Option not available for tip pose dumping thread operations (available: on, off).");
            }
        }
        else if (command.get(0).asString() == "showskeleton"){

            if (command.get(1).asString() == "left") {
                // TODO: implement left hand skeleton.
                reply = Bottle("Not implemented yet! Use 'right' instead.");
            }
            else if (command.get(1).asString() == "right") {

                if (command.get(2).asString() == "on") {

                    thread_dhs_right = new DrawHandSkeletonThread(2, "right", rightarm_remote_driver, rightarm_cartesian_driver, gaze_driver);

                    if (thread_dhs_right != NULL) {
                        reply = Bottle("Starting right hand skeleton drawing thread.");
                        thread_dhs_right->start();
                        showrightskeleton = true;
                    }
                    else {
                        reply = Bottle("Could not initialize right hand skeleton drawing thread.");
                    }

                }
                else if (command.get(1).asString() == "off") {

                    reply = Bottle("Stopping right hand skeleton drawing thread.");

                    thread_dhs_right->stop();
                    delete thread_dhs_right;
                    showrightskeleton = false;

                }
                else {
                    reply = Bottle("Option not available for hand skeleton drawing thread operations (available: on, off).");
                }
            }
            else {
                reply = Bottle("Wrong laterlaity option for hand skeleton drawing thread (available: left, right).");
            }
        }
        else if (command.get(0).asString() == "verbose") {

            if (command.get(1).asString() == "on") {

                verbose = true;
                reply = Bottle("Verbosity is on.");

            }
            else if (command.get(1).asString() == "off") {

                verbose = false;
                reply = Bottle("Verbosity is off.");

            }
            else {
                reply = Bottle("Invalid verbose option (available: on, off).");
            }

        }
        else if (command.get(0).asString() == "quit") {

            reply = Bottle("Quitting...\n");
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
            yInfo() << "Starting finger motion.";
            for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angle_ratio) {
                tip_motion_axis[0] = (center[0] - (radius * sin(alpha)));
                tip_motion_axis[1] = (center[1] + (radius * cos(alpha)));
                yInfo() << "Next position: [" << tip_motion_axis.toString() << "].";
                itf_rightarm_cart->goToPose(tip_motion_axis, tip_motion_angle);
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

        if (dumpdata_ee) {
            thread_wtp->stop();
            delete thread_wtp;
        }
        if (dumpdata_finger) {
            thread_wftp->stop();
            delete thread_wftp;
        }
        if (showrightskeleton) {
            thread_dhs_right->stop();
            delete thread_dhs_right;
        }

        return true;
    }

    bool close()
    {
        yInfo() << "Calling close functions...";

        if (moveendeffectortotip) {
            itf_rightarm_cart->removeTipFrame();
            delete tip_finger;
        }

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
    if (!yarp.checkNetwork())
    {
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
