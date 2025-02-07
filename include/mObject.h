#pragma once

#include <mCommon.h>

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
        Ogre::Vector3 scale = Ogre::Vector3(1.0);
        bool visible = true;
        void setVisible(bool _flag) {visible = _flag;_sNode->setVisible(_flag,false);}
        bool IsVisible() {return visible;}
        bool IsNameMatching(std::string _chName)
        {
            if (childMeshName == _chName)
            {
                return true;
            }
            return false;
        } 
    };
    

    class mObject
    {
    private:
        bool mesh_assigned = false;
        Ogre::SceneNode* astd_Node = nullptr;
        Ogre::SceneNode* dummy_Node = nullptr;
        Ogre::Entity* entityPtr;
    protected:
        Ogre::SceneManager* mScnMgr;
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
        void setPositionLocal(Ogre::Vector3 pos);


        void setRotation(Eigen::Matrix3d &rot);
        void setRotation(double w, double x, double y, double z);
        void setRotation(Ogre::Quaternion qrot);

        void setRotationLocal(double w, double x, double y, double z);
        void setRotationLocal(Ogre::Quaternion qrot);
        void setScale(Ogre::Vector3 &scale);
        void setScale(double _sx, double _sy, double _sz);
        void setVisible(bool _flag);
        // void setMeshFileName(std::string& file_name);
        void setEntityName(std::string& _entity_name);
        void setSceneNode(Ogre::SceneNode* _node);
        void setMaterialColor(Ogre::ColourValue _color);
        void setAxis();

        void createEntity(Ogre::SceneManager* _scnMgr);
        void attachChildMesh(Ogre::SceneManager* _scM, std::string _meshName,Ogre::Vector3 pos,
                             Ogre::Quaternion qrot, Ogre::Vector3 scale = Ogre::Vector3(1.0));
        Ogre::SceneNode* getChildMeshNode(std::string _chMshName);
        bool IsChildMeshExists(std::string &_chName);
        void attachNode(Ogre::SceneNode* _sNode, Ogre::Vector3 pos, Ogre::Quaternion qrot);
        void attachObject(mObject* _Object);
        bool IsChildObjectExists(std::string _chObjName);
        mObject* getChildObject(std::string name);
        void setChildMeshVisible(std::string& _chName, bool _flag);
        void setChildObjectVisible(std::string& _chObjName, bool _flag);
        void setAxisVisible(bool _flag);

        
        // std::string mesh_file_name;
        std::string objectName;
        std::string entity_name;
        std::vector<mChild*> children;
        std::map<std::string,mObject*> child_objects;
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d scale;

        // std::map< std::string, std:: >
    };


    class mRobotLink : public mObject
    {
    private:
    public:
        std::string mesh_file_name;
        mRobotLink(/* args */) {};
        unsigned int type;
        double joint_variable;
        Eigen::Affine3d T_p_l;
        Eigen::Affine3d T_visual;
        Eigen::Vector3d axis;
        Ogre::Vector3 _axis;
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

    class  axis : public mObject
    {
    private:
        Ogre::SceneNode* x_node;
        Ogre::SceneNode* y_node;
        Ogre::SceneNode* z_node;
        void setAxisMaterial(Ogre::Entity* _ent, std::string matName);
    public:
        axis(mObject* _rObject);
        // void setScale(double _sx, double _sy, double _sz);
        ~ axis() {};
    };
    
 // functions.

} // namespace mviz


