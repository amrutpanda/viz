#ifndef _BULLETURDF_IMPORTER_H
#define _BULLETURDF_IMPORTER_H

#include <simCommonHeaders.h>
#include <simCommonRigidBodyBase.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <urdf_model/utils.h>
#include <filesystem>

#include <unistd.h>

class BulletURDFImporter
{
private:
    mMultiBody* m_multibody; 
    btMultiBodyDynamicsWorld* m_world; 
    urdf::ModelInterfaceSharedPtr _urdf;
    urdf::Link* baseLink;
    std::string m_name;
    bool fixedBase = true;
    std::filesystem::path URDF_PATH;
    std::string PACKAGE_PATH;

    btAlignedObjectArray<btQuaternion> rot_world_to_local;
    btAlignedObjectArray<btVector3> local_origin_world_frame;

    void createMultiBody(urdf::Link* _baseLink);
protected:
    btMultiBody* p_multibody;   
    int numLinks;
    int numJoints;
    int _jointNum = 0; // this variable will incremented and used for joint index no.
    std::string resolvePath(std::string& _filePath);
    bool _find_package_path(std::filesystem::path &path, std::filesystem::path& _res);
    bool _find_collision_mesh_world_transform(btMultiBodyLinkCollider*, int linkIndex, btVector3& _colPosThisLinkFrame,
                             btQuaternion& _colRotThisLinkFrame, btTransform& tr);
    bool _find_collision_mesh_local_transform(btMultiBodyLinkCollider*, int, btVector3& , btQuaternion& , btTransform& tr);
    void convertStringToInertiaVector(btMatrix3x3&, btVector3&);
public:
    BulletURDFImporter(): m_world(0) {};
    bool ReadFile(std::string _filename, bool _fixedBase = true);
    void ParseURDFTree(urdf::Link* _rootLink);
    void ParseCollisionMeshes(btMultiBodyDynamicsWorld* m_world);
    void ParseBaseCollisionMesh();
    void addLinkToMultiBody(urdf::Link* , mMultiBody* );
    void createMultiBodyCompFromURDFLink(int , int , urdf::Link* ,float _mass = 0);
    bool createMultiBodyLinkCollisionShapes(int linkIndex, urdf::Link* _link);
    void computePrincipalMomentofInertia(urdf::Link* _ulink, btVector3& _Inertia);
    std::string getName(){return m_name;}
    // void getMultiBodyStruct(mMultiBody* _ptr) {_ptr = m_multibody;}
    mMultiBody* getMultiBodyStruct(btMultiBodyDynamicsWorld* _m_dynamicsWorld);
    
    ~BulletURDFImporter() {};

    btVector3 _base_pos;
    btQuaternion _base_rot;
};

void printVector(btVector3& _v, std::string _str = "");
void printQuaternion(btQuaternion& _v, std::string _str = "");



#endif