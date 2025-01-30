#pragma once
// #include <iostream>
// #include "Ogre.h"
// #include "OgreMesh.h"
#include <mCommon.h>
// #include "eigen3/Eigen/Dense"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <urdf_parser/urdf_parser.h>
// #include "Eigen/Dense"

namespace mviz
{   
    enum Type{
        SPHERE = 0,
        BOX,
        CYLINDER,
        MESH
    };

    struct mChild
    {
        std::string childMeshName;
        std::string childEntityName;
        Ogre::SceneNode* _sNode;
        Ogre::Vector3 rel_pos;
        Ogre::Quaternion rel_qrot;
        bool visible = true;
    };
    

    class mObject
    {
    private:
        bool mesh_assigned = false;
        Ogre::SceneNode* astd_Node = NULL;
        Ogre::Entity* entityPtr;
    public:
        mObject(std::string name,int type);
        mObject() {};
        virtual ~mObject() {};

        Ogre::SceneNode* getSceneNode() {return astd_Node;};
        const Eigen::Vector3d & getPosition();
        const Eigen::Matrix3d & getRotation();
        const Eigen::Vector3d & getScale();
        void setPosition(Ogre::Vector3 pos);
        void setPosition(Eigen::Vector3d &pos);
        void setRotation(Eigen::Matrix3d &rot);
        void setRotation(double w, double x, double y, double z);
        void setRotation(Ogre::Quaternion qrot);
        void setScale(Ogre::Vector3 &scale);
        void setMeshFileName(std::string& file_name);
        void setEntityName(std::string& _entity_name);
        void setSceneNode(Ogre::SceneNode* _node);
        void setMaterialColor(Ogre::ColourValue _color);

        void createEntity(Ogre::SceneManager* _scnMgr);
        void attachChildMesh(Ogre::SceneManager* _scM, std::string _meshName,Ogre::Vector3 pos, Ogre::Quaternion qrot);
        void hookPosition(Ogre::Vector3* _pos);
        
        int type = Type::MESH;
        
        std::string mesh_file_name;
        std::string objectName;
        std::string entity_name;
        std::vector<mChild*> children;
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d scale;

        // std::map< std::string, std:: >
    };


    class mRobotLink : public mObject
    {
    private:
        /* data */
    public:
        mRobotLink(/* args */) {};
        unsigned int type;
        double joint_variable;
        Eigen::Affine3d T_p_l;
        Eigen::Affine3d T_visual;
        Eigen::Vector3d axis;
        // Eigen::Vector3d rpy;
        ~mRobotLink() {};
    };



    class mRobotBase : public mObject
    {
    private:
        /* data */
    public:
        mRobotBase(/* args */) {};
        ~mRobotBase() {};
    };

    class mStaticObject : public mObject
    {
    private:
        /* data */
    public:
        mStaticObject();
        ~mStaticObject();
    };
    
 // functions.

} // namespace mviz


