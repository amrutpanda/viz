#pragma once
// #include <iostream>
// #include "Ogre.h"
// #include "OgreMesh.h"
#include <mCommon.h>
#include "eigen3/Eigen/Dense"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <urdf_parser/urdf_parser.h>

namespace mviz
{   
    enum Type{
        SPHERE = 0,
        BOX,
        CYLINDER,
        MESH
    };

    class mVector : public Eigen::Vector3d
    {
    private:
        /* data */
    public:
        mVector(/* args */) {};
        ~mVector() {};
    };

    class mVector4 : public Eigen::Vector4d
    {
    private:
        /* data */
    public:
        mVector4(/* args */) {};
        ~mVector4() {};
    };
    

    class mMatrix : public Eigen::Matrix3d
    {
    private:
        /* data */
    public:
        mMatrix(/* args */) {};
        ~mMatrix() {};
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

        void assign_mesh(Ogre::Mesh* m);
        const Ogre::Mesh* getMesh();
        const Eigen::Vector3d & getPosition();
        const Eigen::Matrix3d & getRotation();
        const Eigen::Vector3d & getScale();
        void setPosition(Eigen::Vector3d &pos);
        void setRotation(Eigen::Matrix3d &rot);
        void setRotation(double w, double x, double y, double z);
        void setScale(Ogre::Vector3 &scale);
        void setMeshFileName(std::string& file_name);
        void setEntityName(std::string& _entity_name);
        void setSceneNode(Ogre::SceneNode* _node);
        void setMaterialColor(Ogre::ColourValue _color);

        void createEntity(Ogre::SceneManager* _scnMgr);
        
        Ogre::Mesh* mesh;
        int type = Type::MESH;
        
        std::string mesh_file_name;
        std::string objectName;
        std::string entity_name;
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d scale;
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
    void convert_urdf_to_mvector(mVector, urdf::Vector3);

} // namespace mviz


