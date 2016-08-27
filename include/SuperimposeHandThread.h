#ifndef SUPERIMPOSEHANDTHREAD_H
#define SUPERIMPOSEHANDTHREAD_H

#include <yarp/os/ConstString.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Thread.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "SuperimposeHand.h"


class SuperimposeHandThread : public yarp::os::Thread
{
private:
    const yarp::os::ConstString log_ID;
    SuperimposeHand             &sh;
    yarp::os::ResourceFinder    &rf;
    GLFWwindow                  *window;

public:
    SuperimposeHandThread(SuperimposeHand &sh, yarp::os::ResourceFinder &rf, GLFWwindow *window);

    bool threadInit   ();
    void run          ();
    void threadRelease();
};

#endif /* SUPERIMPOSEHANDTHREAD_H */
