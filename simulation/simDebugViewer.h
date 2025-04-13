#ifndef _SIM_DEBUG_VIEWER_H
#define _SIM_DEBUG_VIEWER_H
#include <simMultiBody.h>
#include <mGraphics.h>

using namespace mviz;
class simDebugViewer : public mviz::mGraphics
{
private:
    bool _show_collsion_shape = true;
    simMultiBodyDynamicsWorld* m_world;
    std::vector<std::string> _robot_names;
    std::vector<mMultiBody*> _mulitbody_objects;
    std::vector<std::vector<mObject*>> _robot_objects;
    std::vector<std::vector<mObject*>> _robot_collision_shapes_objects;
    void drawAabb(btTransform& tr, btVector3& _aabbMin, btVector3& _aabbMax);
    void process_mMultibody(mMultiBody* _robot);
    void buildSimRobotGraphicsFromMultiBodyData(mMultiBody* );
    void _createCollisionMeshGraphicalObject(mMultiBody*,int);
    void _processConvexHullShape(btConvexHullShape* _shape, mObject* _mObj, std::string& );

    void updateSimRobotGraphics(int _robot_index);
    void updateSimRobotCollisionGraphics(int _robot_index);
    void attachAabb(btMultibodyLink& , mObject*, btVector3 _color = btVector3(1,1,1));
public:
    simDebugViewer(simMultiBodyDynamicsWorld* _world);
    void renderViewer();
    ~simDebugViewer();

    btAlignedObjectArray<btQuaternion> rot_world_to_local;
    btAlignedObjectArray<btVector3> local_origin_world_frame;
};



#endif
