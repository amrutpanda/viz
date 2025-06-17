#include <ForceSensor.h>

ForceSensor::ForceSensor(btDynamicsWorld* m_world,mMultiBody* _robot,int _link_ind)
{
    m_dynamicsWorld = m_world;
    robot = _robot;
    link_ind = _link_ind;
    // get the joint feedback object.
    _sensor = robot->_jointFeedbackIndexList[link_ind].second;
}

void ForceSensor::enableFilter(double _alpha)
{
    alpha = _alpha;
}

void ForceSensor::updateFilter()
{
    
    Eigen::Vector3d _f,_m;
    btSpatialForceVector f = _sensor->m_reactionForces;
    _f << f.m_topVec.x(), f.m_topVec.y(), f.m_topVec.z();
    _m << f.m_topVec.x(), f.m_topVec.y(), f.m_topVec.z();

    if (_first_loop)
    {
        _f_prev = _f;
        _m_prev = _m;
    }
    
    _f_out = _f_prev* alpha + (1 - alpha)* _f;
    _m_out = _m_prev* alpha + (1 - alpha)* _m;

    _f_prev = _f;
    _m_prev = _m;
}

Eigen::Vector3d ForceSensor::getForce()
{
    updateFilter();
    return _f_out;
}

Eigen::Vector3d ForceSensor::getMoment()
{
    updateFilter();
    return _m_out;
}

void ForceSensor::getForceMoment(Eigen::Vector3d& _force, Eigen::Vector3d& _moment)
{
    updateFilter();
    _force = _f_out;
    _moment = _m_out;
}
