#include <ForceSensor.h>

ForceSensor::ForceSensor(btDynamicsWorld* m_world,mMultiBody* _robot,int _link_ind, double _tstep)
{
    m_dynamicsWorld = m_world;
    robot = _robot;
    link_ind = _link_ind;
    _dt = _tstep;
}

void ForceSensor::updateForceTorque(Eigen::Vector3d& _force, Eigen::Vector3d& _moment)
{
    int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
    std::cout << numManifolds << std::endl;
    
}