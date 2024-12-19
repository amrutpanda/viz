#pragma once

#include "mObject.h"
#include "mGraphics.h"
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <filesystem>

namespace mviz
{
    class mRobot
    {
    private:
        std::string robot_name; // Name of the robot assigned by the user.
        std::string _robot_name; // Name of the robot mentioned in URDF.
        std::string _urdf_file;
        urdf::ModelInterfaceSharedPtr _urdf;
        std::vector <std::string> link_names;
        std::vector < Ogre::SceneNode* > ogreNodes;
        std::vector < mRobotLink*> object_ptrs;
        Ogre::SceneNode* rootNode; 
        // urdf::Link* urdf_link;
        urdf::LinkConstSharedPtr urdf_root_link;

        void convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr, mRobotLink*);
        void convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr, mRobotLink*);
        void createOgreNodeFromLinkSharedPtr(urdf::LinkSharedPtr,Ogre::SceneNode* );
        // void createOgreNodeFromLinkConstSharedPtr(urdf::LinkConstSharedPtr, Ogre::SceneNode*);
    public:
        mRobot(std::string robot_name, std::string urdf_file, Ogre::SceneNode* root_node);
        // mRobot(std::string robot_name, std::string urdf_file): robot_name(robot_name),urdf_file(urdf_file) {};
        ~mRobot() {};

        // void convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode);
    };
    
    
} // namespace mviz

