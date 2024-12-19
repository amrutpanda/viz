#pragma once
#include <iostream>
#include "Ogre.h"
#include "OgreMesh.h"
#include "eigen3/Eigen/Dense"
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
        void setScale(Ogre::Vector3 &scale);
        void setMeshFileName(std::string& file_name);
        void setEntityName(std::string& _entity_name);
        void setSceneNode(Ogre::SceneNode* _node);
        
        Ogre::Mesh* mesh;
        int type = Type::MESH;
        
        std::string mesh_file_name = NULL;
        std::string objectName = NULL;
        std::string entity_name = NULL;
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


