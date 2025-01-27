#pragma once

#include "mObject.h"
// #include "mGraphics.h"
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
// #include <filesystem>

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
        std::vector <std::string> joint_name;
        std::vector <double*> joint_value_holder;   
        std::vector <unsigned int> joint_type;
        std::vector < Ogre::SceneNode* > ogreNodes;
        std::vector < mRobotLink*> object_ptrs;
        Ogre::SceneManager* scnMgr;
        Ogre::SceneNode* rootNode; 

        Eigen::Affine3d T_var;
        // urdf::Link* urdf_link;
        urdf::LinkConstSharedPtr urdf_root_link;

        // void convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr, mRobotLink*);
        // void convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr, mRobotLink*);
        // void createOgreNodesFromLinkSharedPtr(urdf::LinkSharedPtr,Ogre::SceneNode* );


        void convertLinkPtrTomRobotLink(urdf::Link*, mRobotLink*);
        void createOgreNodesFromLinkPtr(urdf::Link*,Ogre::SceneNode* );
        // void createOgreNodeFromLinkConstSharedPtr(urdf::LinkConstSharedPtr, Ogre::SceneNode*);
        void ParseJoint(mRobotLink* _rlink, urdf::JointConstSharedPtr _jptr);
        void ParseVisualInfo(mRobotLink* _rlink, urdf::VisualSharedPtr _vptr);
    public:
        mRobot(std::string robot_name, std::string urdf_file,Ogre::SceneManager* _scnMgr ,Ogre::SceneNode* root_node);
        // mRobot(std::string robot_name, std::string urdf_file): robot_name(robot_name),urdf_file(urdf_file) {};
        std::string getName();

        void updateRobot(Eigen::VectorXd& joint_pos);
        ~mRobot() {};

        // void convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode);
    };
    
    
} // namespace mviz

