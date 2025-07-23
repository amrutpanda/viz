#include <ForceSensor.h>

ForceSensor::ForceSensor(btDynamicsWorld* m_world,mMultiBody* _robot,int _link_ind)
{
    m_dynamicsWorld = m_world;
    robot = _robot;
    link_ind = _link_ind;
    if (link_ind > robot->_multibody->getNumLinks() - 1)
    {
        throw std::runtime_error("Invalid link Index: " + link_ind);
    }
    // change solver setting (experimental)
    m_dynamicsWorld->getSolverInfo().m_splitImpulse = false; // it was false.
    m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -1.0;
    m_dynamicsWorld->getSolverInfo().m_erp = 0.2;
    m_dynamicsWorld->getSolverInfo().m_globalCfm = 1e-4;
    // get the joint feedback object.
    for (auto it : robot->_jointFeedbackIndexList)
    {
        std::cout << "ind: " << it.first << " link_ind: " << _link_ind << std::endl;
        if (it.first == link_ind)
        {
            _sensor = it.second;
            std::cout << "found Sensor link: " << it.first << std::endl;
            break;
        }
    }
    // _sensor = robot->_multibody->getLink(_link_ind).m_jointFeedback;
    if (_sensor == nullptr)
    {
        std::cerr << "Can't attach forcesensor to the link with Index: " << link_ind << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
}

void ForceSensor::enableFilter(double _alpha)
{
    if ( _alpha >= 0 && _alpha <=1)
        _alpha = _alpha;
    else
        throw std::runtime_error(" Alpha value should be between 0 and 1 for filter to work.\n");
}

void ForceSensor::updateFilter()
{
    
    Eigen::Vector3d _f,_m;
    // btSpatialForceVector f = _sensor->m_reactionForces;
    // _f << f.getLinear().x(), f.getLinear().y(), f.getLinear().z();
    // _m << f.getAngular().x(), f.getAngular().y(), f.getAngular().z();
    getContactForce();
    _f = _f_out;
    _m = _m_out;

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

void ForceSensor::getContactForce() // computes force in global frame.
{
    double dt = m_dynamicsWorld->getSolverInfo().m_timeStep;
    int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
    btVector3 _cForce = btVector3(0,0,0);
    btVector3 _cMoment = btVector3(0,0,0);

    btVector3 _cForceLocalFrame = btVector3(0,0,0);
    btVector3 _cMomentLocalFrame = btVector3(0,0,0);
    
    for (int i = 0; i < numManifolds; i++)
    {   
        btPersistentManifold* manifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* _sBody = robot->_multibody->getLinkCollider(link_ind);

        btScalar _force = 0;
        btVector3 _fForce = btVector3(0,0,0);
        btVector3 _normal = btVector3(0,0,0);
        btVector3 _contactPoints = btVector3(0,0,0);

        if (manifold->getNumContacts() == 0)
            continue;
        
        if (_sBody == manifold->getBody0() || _sBody == manifold->getBody1())
        {
            
            for (int j = 0; j < manifold->getNumContacts(); j++)
            {
                btManifoldPoint _mPoint = manifold->getContactPoint(j);
                if (_sBody == manifold->getBody0())
                {
                    _normal += _mPoint.m_normalWorldOnB;
                }
                else if (_sBody == manifold->getBody1())
                {
                    _normal -= _mPoint.m_normalWorldOnB;
                }
                // compute normal forces.
                _force += _mPoint.m_appliedImpulse/dt;
                // lateral forces.
               _fForce += (_mPoint.m_appliedImpulseLateral1 * _mPoint.m_lateralFrictionDir1 +
                            _mPoint.m_appliedImpulseLateral2 * _mPoint.m_lateralFrictionDir2)/dt;

                _contactPoints += manifold->getContactPoint(j).getPositionWorldOnB();
               
                   
            }
        }
        
        // add the total force computed in a manifold to overall total force.
        _cForce += _force* _normal;
        // testing with friction.
        _cForce += _force* _normal + _fForce;
        
        btTransform baseToFsLinkTranform = robot->_multibody->getLink(link_ind).m_cachedWorldTransform;
        _cMoment = (baseToFsLinkTranform.getOrigin() - _contactPoints).cross(_cForce);
    }
    // convert the force to local frame.
    // btTransform world_to_fs =  robot->_multibody->getLink(link_ind).m_cachedWorldTransform.inverse();
    // _cForceLocalFrame = world_to_fs.getBasis()*_cForce;
    _cForceLocalFrame = _cForce;
    _cMomentLocalFrame = _cMoment;

    _f_out <<  _cForceLocalFrame.x(), _cForceLocalFrame.y(), _cForceLocalFrame.z();
    _m_out << _cMomentLocalFrame.x(), _cMomentLocalFrame.y(), _cMomentLocalFrame.z();
    // _m_out << 0, 0, 0;
   
}
