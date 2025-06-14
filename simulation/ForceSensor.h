#ifndef _FORCE_SENSOR_H
#define _FORCE_SENSOR_H

// #ifndef _SIM_COMMON_H
// #define _SIM_COMMON_H
// #include <simCommonHeaders.h>

// #endif
// #include <simCommonHeaders.h>
#include <simCommonRigidBodyBase.h>
#include <Eigen/Dense>

class ForceSensor
{
private:
    btDynamicsWorld* m_dynamicsWorld;
    mMultiBody* robot;
    double _dt = 0.001;
public:
    int link_ind;
    ForceSensor(btDynamicsWorld* m_world, mMultiBody* _robot,int _link_ind, double _tstep);
    ~ForceSensor();

    void updateForceTorque(Eigen::Vector3d& _force, Eigen::Vector3d& _moment);
};

#endif