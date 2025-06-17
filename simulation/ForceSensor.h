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
    btMultiBodyJointFeedback* _sensor;
    double _dt = 0.001;
    double alpha = 0;
    Eigen::Vector3d _f_prev;
    Eigen::Vector3d _m_prev;
    bool _first_loop = true;

    Eigen::Vector3d _f_out;
    Eigen::Vector3d _m_out;
public:
    int link_ind;
    ForceSensor(btDynamicsWorld* m_world, mMultiBody* _robot,int _link_ind);
    ForceSensor():m_dynamicsWorld(nullptr),robot(nullptr) {} ;
    ~ForceSensor();

    /**
     * @brief At this moment the filter used is exponential moving average(EMA)
    */
    void enableFilter(double _alpha);
    void updateFilter();
    Eigen::Vector3d getForce();
    Eigen::Vector3d getMoment();
    void getForceMoment(Eigen::Vector3d& _force, Eigen::Vector3d& _moment);
};

#endif