#include "SuperimposeHandThread.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;

SuperimposeHandThread::SuperimposeHandThread(SuperimposeHand &sh, ResourceFinder &rf, GLFWwindow *window) :
    log_ID("[SuperimposeHandThread]"), sh(sh), rf(rf), window(window) {}

bool SuperimposeHandThread::threadInit() {
    if (!sh.configure(rf)) {
        yError() << log_ID << "RFModule failed to open.";
        return false;
    }

    yInfo() << log_ID << "Setting GL window.";
    sh.setWindow(window);

    yInfo() << log_ID << "RFModule succesfully opened.";

    return true;
}

void SuperimposeHandThread::run() {
    sh.runModule();
}

void SuperimposeHandThread::threadRelease() {
    yInfo() << log_ID << "Releasing.";
}
