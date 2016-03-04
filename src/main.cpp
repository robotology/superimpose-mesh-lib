#include <list>
#include <cmath>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

class WriteTipPoseThread : public Thread {
private:
    unsigned int threadID;

    ICartesianControl *handICart;
    Vector tip_x;
    Vector tip_o;
    BufferedPort<Bottle> tipPoseBPort;

public:

    WriteTipPoseThread(unsigned int threadID, ICartesianControl *&handICart) {
        this->threadID = threadID;
        this->handICart = handICart;
    }

    bool threadInit() {
        yInfo() << "Initializing tip pose dumper for worker " << threadID << ".";

        /* Open buffered port for sending the current tip pose */
        if (!tipPoseBPort.open("/movefinger/tippose:o")) {
            yError() << "Cannot open /movefinger/tippose:o port.";
            return false;
        }

        tip_x.resize(3);
        tip_o.resize(4);

        yInfo() << "Initialization completed for worker " << threadID << ".";

        return true;
    }

    void run() {
        while(!isStopping()) {

            Bottle &tipPoseBottle = tipPoseBPort.prepare();

            handICart->getPose(tip_x, tip_o);

            tipPoseBottle.clear();
            tipPoseBottle.addString(tip_x.toString() + " " + tip_o.toString());

            tipPoseBPort.write();

        }
    }
};

class MoveFinger : public RFModule
{
private:
    ConstString robot;
    bool useGaze;
    bool useTorsoDOF;
    Vector armVel;
    Vector armAcc;
    Vector headVel;
    Vector headAcc;

    Property cartOptions;
    PolyDriver *cartCtrlDriver;
    ICartesianControl *handICart;

    Property controlBoardOptions;
    PolyDriver *ctrlBoardDriver;
    IEncoders *raIEnc;
    IPositionControl2 *raIPos;
    int numRAJoint;
    int numRAEnc;

    Property headOption;
    PolyDriver *headDriver;
    IPositionControl2 *headIPos;
    int numHJoint;

    Property gazeOption;
    PolyDriver *gazeDriver;
    IGazeControl *headIGaze;

    iCubFinger *finger;
    Vector tipChainJoint;
    Matrix tipFrame;
    Vector tip_x;
    Vector tip_o;

    Vector raEncoder;
    list<pair<unsigned int, double>> jointposMap;
    Matrix R;
    Vector init_x;
    Vector init_o;
    Vector initFixation;

    WriteTipPoseThread *wtpThread;
    Port handlerPort;

    bool verbose;
    bool start;
    bool freerunning;

    Vector tipMotionAxis;
    Vector tipMotionAngle;
    Vector center;
    double radius;
    int angleRatio;
    double motionTime;
    double pathTime;

    bool setRightArmRemoteControlboard()
    {
        controlBoardOptions.put("device", "remote_controlboard");
        controlBoardOptions.put("local", "/movefinger/control_right_arm");
        controlBoardOptions.put("remote", "/"+robot+"/right_arm");

        ctrlBoardDriver = new PolyDriver(controlBoardOptions);
        if (ctrlBoardDriver->isValid()) {
            yInfo() << "Right arm remote_controlboard succefully opened.";

            ctrlBoardDriver->view(raIEnc);
            if (!raIEnc) {
                yError() << "Error getting right arm IEncoders interface.\n";
                return false;
            }
            numRAEnc = 0;
            raIEnc->getAxes(&numRAEnc);
            yInfo() << "Right arm encorders succefully read.";

            ctrlBoardDriver->view(raIPos);
            if (!raIPos) {
                yError() << "Error getting right arm IPositionControl2 interface.\n";
                return false;
            }
            yInfo() << "Right arm positions succefully read.";
        } else {
            yError() << "Error opening right arm remote_controlboard device.\n";
            return false;
        }

        numRAJoint = 0;
        raIPos->getAxes(&numRAJoint);
        yInfo() << "Total number of right arm joints: " << numRAJoint << ".";
        Vector tmp(numRAJoint);
        for (int i = 0; i < numRAJoint; ++i) {
            tmp[i] = 10.0;
        }
        if (!raIPos->setRefAccelerations(tmp.data()))
        {
            yError() << "Error setting right arm joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < numRAJoint; ++i) {
            tmp[i] = 15.0;
            if (!raIPos->setRefSpeed(i, tmp[i]))
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
        cartOptions.put("device", "cartesiancontrollerclient");
        cartOptions.put("local", "/movefinger/cart_right_arm");
        cartOptions.put("remote", "/"+robot+"/cartesianController/right_arm");

        cartCtrlDriver = new PolyDriver(cartOptions);
        if (cartCtrlDriver->isValid()) {
            cartCtrlDriver->view(handICart);
            if (!handICart) {
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
        headOption.put("device", "remote_controlboard");
        headOption.put("local", "/movefinger/control_head");
        headOption.put("remote", "/"+robot+"/head");
        headDriver = new PolyDriver(headOption);
        if (headDriver->isValid()) {
            yInfo() << "Head remote_controlboard succefully opened.";

            headDriver->view(headIPos);
            if (!headIPos) {
                yError() << "Error getting head IPositionControl interface.\n";
                return false;
            }
            yInfo() << "Head positions succefully read.";
        } else {
            yError() << "Error opening head remote_controlboard device.";
            return false;
        }

        numHJoint = 0;
        headIPos->getAxes(&numHJoint);
        yInfo() << "Total number of head joints: " << numHJoint << ".";
        Vector tmp(numHJoint);
        for (int i = 0; i < numHJoint; ++i) {
            tmp[i] = 10.0;
        }
        if (!headIPos->setRefAccelerations(tmp.data()))
        {
            yError() << "Error setting head joint accelerations.\n";
            return false;
        }
        for (int i = 0; i < numHJoint; ++i) {
            tmp[i] = 15.0;
            if (!headIPos->setRefSpeed(i, tmp[i]))
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
        gazeOption.put("device", "gazecontrollerclient");
        gazeOption.put("local", "/movefinger/gaze");
        gazeOption.put("remote", "/iKinGazeCtrl");

        gazeDriver = new PolyDriver(gazeOption);
        if (gazeDriver->isValid()) {
            gazeDriver->view(headIGaze);
            if (!headIGaze) {
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
        handICart->getDOF(curDOF);
        yInfo() << "Old DOF: [" + curDOF.toString(0) + "].";
        yInfo() << "Setting iCub to use the DOF from the torso.";
        Vector newDOF(curDOF);
        newDOF[0] = 1;
        newDOF[1] = 2;
        newDOF[2] = 1;
        if (!handICart->setDOF(newDOF, curDOF)) {
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
        Vector encs(numRAEnc);
        if (!raIEnc->getEncoders(encs.data())) {
            yError() << "Cannot read from encoders.";
            return false;
        }
        finger = new iCubFinger("right_index");
        if (!finger->getChainJoints(encs, tipChainJoint)) {
            yError() << "Cannot get joint information to the finger tip.";
            return false;
        }
        tipFrame = finger->getH((M_PI / 180.0) * tipChainJoint);
        tip_x = tipFrame.getCol(3);
        tip_o = dcm2axis(tipFrame);
        if (!handICart->attachTipFrame(tip_x, tip_o)) {
            yError() << "Cannot attach reference frame to finger tip.";
            return false;
        }
        yInfo() << "The end effector reference frame is now onto the right index tip.";

        return true;
    }

    bool setCommandPort()
    {
        yInfo() << "Opening command port.";
        if (!handlerPort.open("/movefinger/command")) {
            yError() << "Cannot open the command port.";
            return false;
        }
        if (!attach(handlerPort)) {
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

        /* Parsing parameters form config file */
        robot = rf.findGroup("PARAMETERS").check("robot", Value("icub")).asString();
        useGaze = rf.findGroup("PARAMETERS").check("usegaze", Value(false)).asBool();
        useTorsoDOF = rf.findGroup("PARAMETERS").check("usetorsodof", Value(true)).asBool();
        if (!rf.findGroup("ARMJOINT").findGroup("vel").isNull() && rf.findGroup("ARMJOINT").findGroup("vel").tail().size() == 16) {
            armVel.resize(16);
            for (int i = 0; i < rf.findGroup("ARMJOINT").findGroup("vel").tail().size(); ++i) {
                armVel[i] = rf.findGroup("ARMJOINT").findGroup("vel").tail().get(i).asDouble();
            }
        }
        yInfo() << armVel.toString();

        /* Right arm control board */
        if (!setRightArmRemoteControlboard()) return false;

        /* Right arm cartesian controler */
        if (!setRightArmCartesianController()) return false;

        /* Head control board */
        if (!setHeadRemoteControlboard()) return false;

        /* Gaze control */
        if (useGaze && !setGazeController()) return false;

        /* Enable torso DOF */
        if (useTorsoDOF && !setTorsoDOF()) return false;

        /* Move reference framework to the right index tip */
        if (!setTipFrame()) return false;

        /* Set initial configuration and pose of the hand */
        yInfo() << "Closing fingers.";
        raEncoder.resize(numRAEnc);
        raIEnc->getEncoders(raEncoder.data());
        jointposMap = {{13, 80},
                       {14, 150},
                       {15, 180},
                       {8, 80},
                       {9, 10},
                       {10, 80}};
        for (auto num = jointposMap.cbegin(); num != jointposMap.cend(); ++num) {
            yInfo() << "Moving joint " << num->first << " to the position " << num->second << ".";
            if (abs(raEncoder[num->first] - num->second) > 3) {
                raEncoder[num->first] = num->second;
                raIPos->positionMove(raEncoder.data());
                bool done = false;
                while (!done) {
                    raIPos->checkMotionDone(&done);
                    Time::delay(0.25);
                }
            }
        };
        yInfo() << "Fingers succesfully closed.";
        yInfo() << "Moving hand to the initial position.";
        R.resize(3, 3);
        R(0,0) = -1.0;   R(0,1) =  0.0;   R(0,2) =  0.0;
        R(1,0) =  0.0;   R(1,1) =  1.0;   R(1,2) =  0.0;
        R(2,0) =  0.0;   R(2,1) =  0.0;   R(2,2) = -1.0;
        init_o = dcm2axis(R);
        init_x.resize(3);
        init_x[0] = -0.35;
        init_x[1] = +0.20;
        init_x[2] = +0.20;
        handICart->goToPoseSync(init_x, init_o);
        handICart->waitMotionDone();
        yInfo() << "The hand is in position.";

        /* Set initial fixation point */
        if (useGaze) {
            Vector tmp;
            headIGaze->getFixationPoint(tmp);
            initFixation = init_x;
            initFixation[0] -= 0.05;
            initFixation[1] -= 0.05;
            if (norm(tmp - initFixation) > 3) {
                yInfo() << "Moving head to initial fixation point: [" << initFixation.toString() << "].";
                headIGaze->lookAtFixationPoint(initFixation);
                headIGaze->waitMotionDone();
            }
            yInfo() << "Gaze motion done.";
        }

        /* Set initial finger motion point */
        tipMotionAxis.resize(3);
        tipMotionAxis[2] = init_x[2];
        tipMotionAngle = init_o;
        radius = 0.10;
        center.resize(2);
        center[0] = init_x[0];
        center[1] = init_x[1] - radius;
        angleRatio = 12;
        motionTime = 10.0;
        pathTime = motionTime / angleRatio;
        handICart->setTrajTime(pathTime);

        /* Initialize tip pose dumper thread */
        wtpThread = new WriteTipPoseThread(1, handICart);
        wtpThread->start();

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
                reply = Bottle("Star freerunning motion mode.");
            } else {
                reply = Bottle("Starting single finger motion.");
            }

        }
        else if (command.get(0).asString() == "stop") {
            start = false;

            if (freerunning) freerunning = false;
            reply = Bottle("Stopping finger motion.");

        }
        else if (command.get(0).asString() == "verbose") {

            if (command.get(1).asString() == "on") {
                verbose = true;
                reply = Bottle("Verbosity is on.");
            }
            else if (command.get(1).asString() == "off") {
                verbose = false;
                reply = Bottle("Verbositi is off.");
            }
            else {
                reply = Bottle("Invalid verbose key. Verbose = "+((verbose)? ConstString("ON"):ConstString("OFF"))+".");
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
            yInfo() << "Starting finger motion.";
            for (double alpha = 0.0; alpha < (2* M_PI); alpha += M_PI / angleRatio) {
                tipMotionAxis[0] = (center[0] - (radius * sin(alpha)));
                tipMotionAxis[1] = (center[1] + (radius * cos(alpha)));
                yInfo() << "Next position: [" << tipMotionAxis.toString() << "].";
                handICart->goToPose(tipMotionAxis, tipMotionAngle);
                Time::delay(0.7 * pathTime);
            }
            yInfo() << "Motion done.";
            if (!freerunning) start = false;
        }
        return true;
    }


    bool interruptModule()
    {
        yInfo() << "Interrupting the module...";
        wtpThread->stop();
        return true;
    }

    bool close()
    {
        yInfo() << "Calling close function...";
        handICart->removeTipFrame();
        if (cartCtrlDriver->isValid()) cartCtrlDriver->close();
        if (ctrlBoardDriver->isValid()) ctrlBoardDriver->close();
        if (headDriver->isValid()) headDriver->close();
        if (gazeDriver->isValid()) gazeDriver->close();
        if (handlerPort.isOpen()) handlerPort.close();

        yInfo() << "Deallocating memory...";
        delete finger;
        delete cartCtrlDriver;
        delete ctrlBoardDriver;
        delete headDriver;
        delete gazeDriver;
        delete wtpThread;

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
