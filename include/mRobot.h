#ifndef _MROBOT_H
#define _MROBOT_H
#pragma once

#include "mObject.h"
// #include "axis.h"
// #include "mGraphics.h"
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
// #include <filesystem>

namespace mviz
{
    enum AXIS
    {
        X = 0,
        Y,
        Z
    };

    class mRobot
    {
    private:
        bool _collision_flag = false;
        std::string robot_name; // Name of the robot assigned by the user.
        std::string _robot_name; // Name of the robot mentioned in URDF.
        std::string _urdf_file;
        std::filesystem::path URDF_PATH;
        std::string PACKAGE_PATH;
        // urdf::ModelInterfaceSharedPtr _urdf;
        std::vector <std::string> link_names;
        std::vector <std::string> joint_names;
        std::vector <double*> joint_value_holder;   
        std::vector <unsigned int> joint_type;
        std::vector < Ogre::SceneNode* > ogreNodes;
        std::vector < mRobotLink*> object_ptrs;
        Ogre::SceneManager* scnMgr;
        Ogre::SceneNode* rootNode; 

        Eigen::Affine3d T_var;
        Eigen::Vector3d base_pos;
        Eigen::Quaterniond base_rot;
        // urdf::Link* urdf_link;
        urdf::LinkConstSharedPtr urdf_root_link;

        double mfAngle = 90;          // mesh rotation angle. Applicable for some DAE meshes.
        Ogre::Vector3 mfAxis = Ogre::Vector3::UNIT_X;    // mesh rotation axis. Applicable for some DAE meshes

        // void convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr, mRobotLink*);
        // void convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr, mRobotLink*);
        // void createOgreNodesFromLinkSharedPtr(urdf::LinkSharedPtr,Ogre::SceneNode* );


        void convertVisualLinkPtrTomRobotLink(urdf::Link*, mRobotLink*);
        void convertCollisionLinkPtrTomRobotLink(urdf::Link*, mRobotLink*);
        void createOgreNodesFromLinkPtr(urdf::Link*,Ogre::SceneNode* );
        void createOgreNodesFromLinkPtr(urdf::Link*,Ogre::SceneNode* , bool);
        // void createOgreNodeFromLinkConstSharedPtr(urdf::LinkConstSharedPtr, Ogre::SceneNode*);
        void ParseJoint(mRobotLink* _rlink, urdf::JointConstSharedPtr _jptr);
        void ParseVisualInfo(mRobotLink* _rlink, urdf::VisualSharedPtr _vptr);
        void ParseBaseLink(mRobotLink* _rlink);
        unsigned int getUrdfGeometryType(urdf::GeometrySharedPtr _gptr);
        std::string resolvePath(std::string& _filePath);
        bool _find_package_path(std::filesystem::path &path, std::filesystem::path& _res);
    public:
        // mRobot(std::string robot_name, std::string urdf_file,Ogre::SceneManager* _scnMgr ,Ogre::SceneNode* root_node);
        mRobot(std::string robot_name, std::string urdf_file,Ogre::SceneManager* _scnMgr ,
            Ogre::SceneNode* root_node, Eigen::Vector3d _bpos = Eigen::Vector3d::Zero(), 
            Eigen::Quaterniond _brot = Eigen::Quaterniond::Identity(), bool show_collision = false );

        // mRobot(std::string robot_name, std::string urdf_file): robot_name(robot_name),urdf_file(urdf_file) {};
        std::string getName();

        void updateRobot(Eigen::VectorXd& joint_pos);
        void setBasePose(Eigen::Vector3d _pose);
        void setBaseRotation(Eigen::Quaterniond _qRotation);
        void setRobotAxisVisible(bool _flag);
        int getRobotNumJoints();
        void flipDAEMeshes(double& angle, int axis); // Angle is in Degree
        void printRobotJointNames();
        mRobotLink* getRobotLinkFromFrameName(std::string& _fName);

        ~mRobot();

        // void convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode);
    };
    
    
} // namespace mviz

#endif