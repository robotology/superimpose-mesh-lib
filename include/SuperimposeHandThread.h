#ifndef SUPERIMPOSEHANDTHREAD_H
#define SUPERIMPOSEHANDTHREAD_H

#include <yarp/os/all.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "SuperimposeHand.h"

using namespace yarp::os;

class SuperimposeHandThread : public Thread
{
private:
    const ConstString log_ID;
    SuperimposeHand &sh;
    ResourceFinder &rf;
    GLFWwindow *window;

public:
    SuperimposeHandThread(SuperimposeHand &sh, ResourceFinder &rf, GLFWwindow *window);

    bool threadInit   ();
    void run          ();
    void threadRelease();
};

#endif /* SUPERIMPOSEHANDTHREAD_H */
