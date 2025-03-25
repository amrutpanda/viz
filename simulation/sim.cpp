#include <sim.h>

Simulation::Simulation(/* args */)
{
    // intiate logger.
    Logger logger("simlog");
    log = &logger;
    log->info("created logger");
    // initiate bullet collision and dynamics instances.
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,0,0));
}

void Simulation::setGravity(double _gx, double _gy, double _gz)
{
    dynamicsWorld->setGravity(btVector3(_gx,_gy,_gz));
}

unsigned int Simulation::addBodyBox(std::string _name, double l, double b, double h, double m, 
                            Eigen::Vector3d _pose, Eigen::Quaterniond _q)
{
    btCollisionShape* boxShape = new btBoxShape(btVector3(l,b,h));
    // save collision shapes.
    collisionShapes.push_back(boxShape);
    if (!checkIfrBodyExists(_name))
        rigidBodyNames.push_back(_name);
    else
        throw std::runtime_error("Rigid Body with given name already exists. Name: "+ _name);

    // setup transform.
    btTransform Transform;
    Transform.setIdentity();
    Transform.setOrigin(btVector3(_pose.x(),_pose.y(),_pose.z()));
    btQuaternion rot(_q.x(),_q.y(),_q.z(),_q.w());
    Transform.setRotation(rot);

    btScalar mass(m);
    btVector3 localInertia(0, 0, 0);
    bool isDynamics = (mass != 0.f);
    if(!isDynamics)
        std::cout << "This Rigid body is Static. Name: " << _name << std::endl;
    else
    {
        boxShape->calculateLocalInertia(mass,localInertia);
    }

    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _MotionStates.push_back(myMotionState);
    // construct a rigid body.
    btRigidBody::btRigidBodyConstructionInfo rbinfo(mass,myMotionState,boxShape,localInertia);
    btRigidBody* body = new btRigidBody(rbinfo);

    body->setRestitution(0.5);

    _rigidBodies.push_back(body);
    // add rigid body to the dynamicsWorld.
    dynamicsWorld->addRigidBody(body);
    // return the index of the object.
    return rigidBodyNames.size() - 1 ;
}

unsigned int Simulation::addBodySphere(std::string _name, double r, double m,
                        Eigen::Vector3d _pose, Eigen::Quaterniond _q)
{
    btCollisionShape* sphereShape = new btSphereShape(btScalar(r));
    // save collision shapes.
    collisionShapes.push_back(sphereShape);
    if (!checkIfrBodyExists(_name))
        rigidBodyNames.push_back(_name);
    else
        throw std::runtime_error("Rigid Body with given name already exists. Name: "+ _name);
    // setup transform.
    btTransform Transform;
    Transform.setIdentity();
    Transform.setOrigin(btVector3(_pose.x(),_pose.y(),_pose.z()));
    btQuaternion rot(_q.x(),_q.y(),_q.z(),_q.w());
    Transform.setRotation(rot);

    btScalar mass(m);
    btVector3 localInertia(0, 0, 0);
    bool isDynamics = (mass != 0.f);
    if(!isDynamics)
        std::cout << "This Rigid body is Static. Name: " << _name << std::endl;
    else
    {
        sphereShape->calculateLocalInertia(mass,localInertia);
    }

    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _MotionStates.push_back(myMotionState);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sphereShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    body->setRestitution(0.5);

    dynamicsWorld->addRigidBody(body);
    _rigidBodies.push_back(body);
     // return the index of the object.
    return rigidBodyNames.size() - 1 ;
}

void Simulation::applyForce(int bodyIndex, Eigen::Vector3d _force)
{
    btRigidBody* rbody = _rigidBodies[bodyIndex];
    btVector3 _rel_pos,_f(_force.x(), _force.y(), _force.z());

    btTransform trans;
    rbody->getMotionState()->getWorldTransform(trans);
    _rel_pos = trans.getOrigin();
    rbody->applyForce(_f,_rel_pos);
    
}

void Simulation::stepSimulation(double _timesteps)
{
    if (first_loop)
    {
        if (_timesteps < _fixedTimeStep)
            _fixedTimeStep = _timesteps;
            _maxSubsteps = int(_timesteps/_fixedTimeStep) + 1;
            // _maxSubsteps = 1;
    }

    dynamicsWorld->stepSimulation(_timesteps,_maxSubsteps,_fixedTimeStep);
    
}

void Simulation::close()
{
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
    for (int i = 0; i < collisionShapes.size(); i++)
    {
        delete collisionShapes[i];
        delete _rigidBodies[i];
        delete _MotionStates[i];
    }
    memReleased = true;
}

void Simulation::getBodyPoseAndRotation(int bodyIndex, Eigen::Vector3d& _pos, Eigen::Quaterniond& _q)
{
    btRigidBody* _rbody = _rigidBodies[bodyIndex];
    btTransform trans;
    if (_rbody && _rbody->getMotionState())
    {
        _rbody->getMotionState()->getWorldTransform(trans);
    }

    _pos.x() = trans.getOrigin().getX();
    _pos.y() = trans.getOrigin().getY();
    _pos.z() = trans.getOrigin().getZ();

    _q.w() = trans.getRotation().getW();
    _q.x() = trans.getRotation().getX();
    _q.y() = trans.getRotation().getY();
    _q.z() = trans.getRotation().getZ();
    
}

Simulation::~Simulation()
{
    if (!memReleased)
        close();
    // delete collisionConfiguration;
    // delete dispatcher;
    // delete overlappingPairCache;
    // delete solver;
    // delete dynamicsWorld;
    // for (int i = 0; i < collisionShapes.size(); i++)
    // {
    //     delete collisionShapes[i];
    // }
    
    // // std::cout << "Released all memory" << std::endl;
    // log->debug("Released all dynamically allocated memory");
}

bool Simulation::checkIfrBodyExists(std::string _name)
{
    for (int i = 0; i < rigidBodyNames.size(); i++)
    {
        if (rigidBodyNames[i] == _name)
        {
            return true;
        }
    }
    return false;   
}