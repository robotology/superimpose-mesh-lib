# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Claudio Fantacci
# CopyPolicy: Released under the terms of the GNU GPL v3.0.
#
# idl_mesh.thrift

/**
 * SuperimposeHandCADIDL
 *
 * IDL Interface to \ref superimpose-hand services.
 */

service SuperimposeHandCADIDL
{
    /**
     * View or hide background image of the mesh window.
     * @param status true/false to turn background on/off.
     * @return true activation/deactivation success, false otherwise.
     */
    bool mesh_background(1:bool status);

    /**
     * Render the mesh as a wireframe. If disabled (default behaviour)
     * the mesh is filled without light.
     * @param status true/false to turn wireframe rendering on/off.
     * @return true activation/deactivation success, false otherwise.
     */
    bool mesh_wireframe(1:bool status);

    /**
     * Set mipmaps color filtering technique.
     * Default is "nearest" (i.e. nearest neighbor).
     * @param type can be "nearest" (default) or "linear".
     * @return true on correct activation, false otherwise.
     */
    bool mesh_mipmaps(1:string type);
}
