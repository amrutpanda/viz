#pragma once

#include "mObject.h"
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <filesystem>

namespace mviz
{
    class mRobot
    {
    private:
        std::string robot_name;
        std::string _robot_name; // Name of the robot mentioned in URDF.
        std::string _urdf_file;
        urdf::ModelInterfaceSharedPtr _urdf;
        std::vector <std::string> link_names;
        std::vector < Ogre::SceneNode* > ogreNodes;
        Ogre::SceneNode* rootNode; 
        // urdf::Link* urdf_link;
        urdf::LinkConstSharedPtr urdf_link;

        void convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr, mRobotLink*);
        void convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr, mRobotLink*);
    public:
        mRobot(std::string robot_name, std::string urdf_file, Ogre::SceneNode* root_node);
        // mRobot(std::string robot_name, std::string urdf_file): robot_name(robot_name),urdf_file(urdf_file) {};
        ~mRobot() {};

        // void convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode);
    };
    
    
} // namespace mviz

