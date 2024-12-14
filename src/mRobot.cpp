#include "mRobot.h"


namespace mviz
{
    mRobot::mRobot(std::string robotname, std::string urdf_file, Ogre::SceneNode* root_node)
    {
        robot_name = robotname;
        _urdf_file = urdf_file;
        rootNode = root_node;
        _urdf = urdf::parseURDFFile(_urdf_file);

        // urdf::LinkConstSharedPtr linkPtr = _urdf->getRoot();
        // urdf_link = linkP

        urdf_link = _urdf->getRoot();
        
        // convertRobotLinkToOgreNode(urdf_link, rootNode);
    }

    // void mRobot::convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode)
    // {
    //     // To be implemented.
    // }

    void mRobot::convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr ulink, mRobotLink* rlink)
    {
        
        urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
        if (m == NULL)
        {
            std::cout << "At this moment only Mesh objects are supported." << std::endl;
            throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

        }

        rlink->type = Type::MESH; // this is redundant.
        rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();
        
        rlink->scale << m->scale.x , m->scale.y, m->scale.z;
    }




} // namespace mviz
