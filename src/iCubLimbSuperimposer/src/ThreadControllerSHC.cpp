#include "iCubLimbSuperimposer/ThreadControllerSHC.h"

#include <yarp/os/LogStream.h>


bool ThreadControllerSHC::mesh_background(const bool status)
{
    if (status && !mesh_back) {
        yInfo() << log_ID << "Enabling background of the mesh window.";

        mesh_back = true;

        return true;
    } else if (!status && mesh_back) {
        yInfo() << log_ID << "Disabling background of the mesh window.";

        mesh_back = false;

        return true;
    } else return false;
}


bool ThreadControllerSHC::mesh_wireframe(const bool status)
{
    if (status && !mesh_wires) {
        yInfo() << log_ID << "Enabling wireframe rendering.";

        mesh_wires = true;

        return true;
    } else if (!status && mesh_wires) {
        yInfo() << log_ID << "Disabling wireframe rendering.";

        mesh_wires = false;

        return true;
    } else return false;
}


bool ThreadControllerSHC::mesh_mipmaps(const std::string& type)
{
    if (type == "nearest") {
        yInfo() << log_ID << "Setting mipmaps color filtering to nearest neighbor.";

        mesh_mmaps = NEAREST;

        return true;
    } else if (type == "linear") {
        yInfo() << log_ID << "Setting mipmaps color filtering to linear.";

        mesh_mmaps = LINEAR;

        return true;
    } else return false;
}


ThreadControllerSHC::ThreadControllerSHC() : log_ID("[ThreadControllerSHC]"), mesh_back(true), mesh_wires(true), mesh_mmaps(NEAREST) {};
