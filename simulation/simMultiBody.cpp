#include <simMultiBody.h>

simMultiBodyDynamicsWorld::simMultiBodyDynamicsWorld()
{
    //code.
    // _robot_list.resize(10);
    // _robot_mtbs.resize(10);
    InitialiseDynamicsWorld();
}

void simMultiBodyDynamicsWorld::LoadFromWorldFile(std::string _world_file)
{
    std::filesystem::path _path = std::filesystem::canonical(_world_file);
    if (_path.extension() != ".world")
        throw std::runtime_error("Loading failed. Please load a file with extension '.world'...");
    // pugixml parsing
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(_world_file.c_str());
    if (!result)
        throw std::runtime_error("Error while reading the WORLD file.\n");
    pugi::xml_node root_node = doc.root().child("world");

    for (const pugi::xml_node _node : root_node)
    {
        pugi::xml_node tNode;
        std::string name, type, filename, tval;
        btVector3 _xyz;
        btQuaternion _rpy;
        btQuaternion _q;
        btVector3 _inertia;
        btVector3 _scale;
        Eigen::Vector3d xyz;
        Eigen::Quaterniond rpy;
        Eigen::Quaterniond q;
        Eigen::Vector3d inertia;
        Eigen::Vector3d scale;
        double l,b,h,r;
        std::string _node_name = _node.name();
        if (_node_name == "robot")
        {
            name = _node.attribute("name").value();
            /**
             * Robot name should match the name in URDF file.
             */
            std::cout << "sim robot name = " << name << std::endl;
            filename = _node.child("path").first_attribute().value();
    
            convertStringTobtVector3(_node.child("origin").attribute("rpy").value(),_xyz);
            convertStringTobtQuaternion(_node.child("origin").attribute("rpy").value(),_rpy);
            
            xyz = Eigen::Vector3d(_xyz.x(), _xyz.y(), _xyz.z());
            rpy = Eigen::Quaterniond(_rpy.w(),_rpy.x(), _rpy.y(), _rpy.z());

            LoadRobotFromURDFFile(filename,xyz,rpy,true,false);
        }
        else if (_node_name == "object")
        {
            name = _node.attribute("name").value();
            type = _node.attribute("type").value();

            convertStringTobtVector3(_node.child("origin").attribute("rpy").value(),_xyz);
            convertStringTobtQuaternion(_node.child("origin").attribute("rpy").value(),_rpy);
            
            xyz = Eigen::Vector3d(_xyz.x(), _xyz.y(), _xyz.z());
            rpy = Eigen::Quaterniond(_rpy.w(),_rpy.x(), _rpy.y(), _rpy.z());

            double mass = std::stod( _node.child("inertial").attribute("mass").value() );
            convertStringTobtVector3(_node.child("inertial").attribute("inertia").value(), _inertia);
            inertia = Eigen::Vector3d(_inertia.x(), _inertia.y(), _inertia.z());

            if (type == "mesh")
            {
                filename = _node.child("mesh").attribute("filename").value();
                convertStringTobtVector3(_node.child("mesh").attribute("scale").value(), _scale);
                scale = Eigen::Vector3d(_scale.x(), _scale.y(), _scale.z());
                addBodyConvexHull(filename,mass,inertia,xyz,rpy,scale);
            }
            else if (type == "box")
            {
                l = _node.child("dim").attribute("l").as_double();
                b = _node.child("dim").attribute("b").as_double();
                h = _node.child("dim").attribute("h").as_double();
                std::cout << "creating sim box\n";
                addBodyBox(l,b,h,mass,xyz,rpy);
            }
            else if (type == "cylinder")
            {
                r = _node.child("dim").attribute("radius").as_double();
                h = _node.child("dim").attribute("height").as_double();
                std::cout << "creating sim cylinder.\n";
                addBodyCylinder(r,h,mass,xyz,rpy);
            }
            else if(type == "sphere")
            {
                r = _node.child("dim").attribute("radius").as_double();
                std::cout << "creating sim sphere\n";
                addBodySphere(r,mass,xyz,rpy);
            }
        }
        else
        {
            /* code */
        }
        
    }
    
}

simMultiBodyDynamicsWorld::~simMultiBodyDynamicsWorld()
{
    if (m_solverinterface != nullptr)
        delete m_solverinterface;

    if (_multibody_name_map.size() != 0)
    {
        for (auto it : _multibody_name_map)
        {
            delete it.second;
        }
    }

    // if (_rigidbody_name_map.size() != 0)
    // {
    //     for (auto it : _multibody_name_map)
    //     {
    //         delete it.second;
    //     }
    // }

    for (int i = 0; i < _rigidBodyList.size(); i++)
    {
        if (_rigidBodyList[i] != nullptr)
            delete _rigidBodyList[i];
        if (_rigidBodyCollisionShapes[i] != nullptr)
            delete _rigidBodyCollisionShapes[i];
        if(_rigidBodyMotionStates[i] != nullptr)
            delete _rigidBodyMotionStates[i];
    }

    // clear forcesensor objects.
    for (auto it : _ft_sensors)
    {
        delete it;
    }
    
    std::cout << "cleared simMultiBodyDynamicsWorld\n";
    
}


void simMultiBodyDynamicsWorld::InitialiseDynamicsWorld()
{
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();

    btMultiBodyConstraintSolver* sol;
    btMLCPSolverInterface* mlcp;

    switch (g_constraintSolverType)
    {
    case 0:
        sol = new btMultiBodyConstraintSolver;
        std::cout << "Constraint Solver: Sequential Impulse." << std::endl;
        break;
    case 1:
        mlcp = new btSolveProjectedGaussSeidel();
        sol = new btMultiBodyMLCPConstraintSolver(mlcp);
        std::cout << "Constraint Solver: MLCP + PGS" << std::endl;
        break;
    case 2:
        mlcp = new btDantzigSolver();
        sol = new btMultiBodyMLCPConstraintSolver(mlcp);
        std::cout << "Constraint Solver: MLCP + Dantzig" << std::endl;
        break;
    default:
        mlcp = new btLemkeSolver();
        sol = new btMultiBodyMLCPConstraintSolver(mlcp);
        std::cout << "Constraint Solver: MLCP + Lemke" << std::endl;
        break;
    }
    
    m_solver = sol;
    m_solverinterface = mlcp;
    m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase,sol,m_collisionConfiguration);
    m_dynamicsWorld->getSolverInfo().m_jointFeedbackInJointFrame = true;
    m_dynamicsWorld->getSolverInfo().m_jointFeedbackInWorldSpace = true;
    // setGravity(0,0, -10);
}

void simMultiBodyDynamicsWorld::LoadRobotFromURDFFile(std::string _filename, Eigen::Vector3d _base_pose,
                                Eigen::Quaterniond _base_rotation, bool _fixedBase, bool _has_selfcollision)
{
    BulletURDFImporter importer;
    if (!importer.ReadFile(_filename,_fixedBase))
    {
        throw std::runtime_error("Error while reading file: " + _filename);
    }
    _multibody_name_map[importer.getName()] = importer.getMultiBodyStruct(m_dynamicsWorld);
    m_dynamicsWorld->addMultiBody(_multibody_name_map.at(importer.getName())->_multibody);
    
    _multibody_name_map.at(importer.getName())->_multibody->setHasSelfCollision(_has_selfcollision); // flag for checking self-collision.
    
    // updating the tranforms (testing)
    // _multibody_name_map.at(importer.getName())->updateTransforms();
    _multibody_name_map.at(importer.getName())->setupCollisionFlags();

    setRobotBasePose(importer.getName(),_base_pose.x(), _base_pose.y(), _base_pose.z());
    setRobotBaseOrientation(importer.getName(),_base_rotation.x(), _base_rotation.y(), _base_rotation.z(), _base_rotation.w());
    
    // _multibody_name_map.at(importer.getName())->_multibody->
    std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
    std::cout << "Robot Name: " << importer.getName() << std::endl;
    std::cout << "Num of DOFs: " <<_multibody_name_map.at(importer.getName())->_multibody->getNumDofs() << std::endl;
    std::cout << "Num of Links: " << _multibody_name_map.at(importer.getName())->_multibody->getNumLinks() << std::endl;
    std::cout << "Num of mLinks: " << m_dynamicsWorld->getMultiBody(0)->getNumLinks() << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;   

}

void simMultiBodyDynamicsWorld::setRobotBasePose(std::string _robotName, double _x, double _y, double _z)
{
    btMultiBody* _robot = _multibody_name_map.at(_robotName)->_multibody;
    
    _robot->setBasePos(btVector3(_x,_y,_z));
    _robot->setBaseVel(btVector3(0, 0, 0));
}

void simMultiBodyDynamicsWorld::setRobotBaseOrientation(std::string _robotName, double _qx, double _qy, 
                                                        double _qz, double _qw)
{
    btMultiBody* _robot = _multibody_name_map.at(_robotName)->_multibody;
    _robot->setWorldToBaseRot(btQuaternion(_qx,_qy,_qz,_qw));
}

void simMultiBodyDynamicsWorld::getRobotJointInfo(std::string _robotName)
{
    mMultiBody* mbs;
    try
    {
        mbs = _multibody_name_map[_robotName];
        for (int i = 0; i < mbs->_jointNameIndexList.size(); i++)
        {
            std::cout << "Index: " << mbs->_jointNameIndexList[i].first << "  JointName: " 
                                    << mbs->_jointNameIndexList[i].second << std::endl;
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        throw std::runtime_error("getRobotJointInfo: robot name doesnot exist: name "+_robotName);
    }
}

btMultiBody* simMultiBodyDynamicsWorld::getRobotObject(std::string _robotName)
{
    mMultiBody* mbs;
    try
    {
        mbs = _multibody_name_map[_robotName];
        std::cout << mbs->_multibody->getNumDofs() << std::endl;
        return mbs->_multibody;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        // throw std::runtime_error("getRobotJointInfo: robot name doesnot exist: name "+_robotName);
    }
    return nullptr;
}

btMultiBody* simMultiBodyDynamicsWorld::getRobotObject(int index)
{
    if (index >= _multibody_name_map.size())
        throw std::runtime_error("Invalid index: (getRobotJointPos): " + std::to_string(index));
    // _multibody_name_map
    mMultiBody* _mbs;
    int _count = 0;
    for (auto it = _multibody_name_map.begin() ; it != _multibody_name_map.end() ; ++it)
    {
        if (_count == index)
        {
            _mbs = it->second;
            break;
        }
        _count++;
    }
    return _mbs->_multibody;
}

void simMultiBodyDynamicsWorld::getRobotJointPos(int index, Eigen::VectorXd& _q)
{
    if (index >= _multibody_name_map.size())
        throw std::runtime_error("Invalid index: (getRobotJointPos): " + std::to_string(index));
    // _multibody_name_map
    mMultiBody* _mbs;
    int _count = 0;
    for (auto it = _multibody_name_map.begin() ; it != _multibody_name_map.end() ; ++it)
    {
        if (_count == index)
        {
            _mbs = it->second;
            assert((_q.size() == _mbs->_jointNameIndexList.size(),"Mismatch in vector size to joint number. Inside getRobotJointPos\n"));
            break;
        }
        _count++;
    }

    btMultiBody* mb = _mbs->_multibody;
    for (int i = 0; i < _mbs->_jointNameIndexList.size(); i++)
    {
        _q[i] = mb->getJointPos(i);
    } 
    
}

btConstraintSolver* simMultiBodyDynamicsWorld::getSolver()
{
    return m_solver;
}

btMultiBodyConstraintSolver* simMultiBodyDynamicsWorld::getMultiBodySolver()
{
    return dynamic_cast<btMultiBodyConstraintSolver*>(m_solver);
}

mMultiBody* simMultiBodyDynamicsWorld::getMultiBodyObject(int index)
{
    if (index >= _multibody_name_map.size())
        throw std::runtime_error("Invalid index: (getMultiBodyObject): " + std::to_string(index));
    // _multibody_name_map
    mMultiBody* _mbs;
    int _count = 0;
    for (auto it = _multibody_name_map.begin() ; it != _multibody_name_map.end() ; ++it)
    {
        if (_count == index)
        {
            _mbs = it->second;
            break;
        }
        _count++;
    }
    return _mbs;
}

mMultiBody* simMultiBodyDynamicsWorld::getMultiBodyObject(std::string _name) noexcept
{
    mMultiBody* _mbs = nullptr;
    for (auto it = _multibody_name_map.begin() ; it != _multibody_name_map.end(); ++it)
    {
        if (it->first == _name)
        {
            _mbs = it->second;
            break;
        }
    }
    return _mbs;
}

int simMultiBodyDynamicsWorld::getNumRobots() noexcept
{
    return _multibody_name_map.size();
}

void simMultiBodyDynamicsWorld::getRobotJointPos(mMultiBody* _robot,Eigen::VectorXd& _q)
{
    if (_q.size() != _robot->_multibody->getNumDofs())
        throw std::runtime_error("vector size doesnot match with the no. of joints. Inside: getRobotJointPos function\n");
    for (int i = 0; i < _robot->_multibody->getNumDofs(); i++)
    {
        // _q[i] = _robot->_multibody->getJointPos(i);
        _q[i] = _robot->_multibody->getJointPos(_robot->_jointNameIndexList[i].first);
    }
}

void simMultiBodyDynamicsWorld::getRobotJointVel(mMultiBody* _robot,Eigen::VectorXd& _q)
{
    if (_q.size() != _robot->_multibody->getNumDofs())
        throw std::runtime_error("vector size doesnot match with the no. of joints.Inside: getRobotJointVel function\n");
    for (int i = 0; i < _robot->_multibody->getNumDofs(); i++)
    {
        // _q[i] = _robot->_multibody->getJointVel(i);
        _q[i] = _robot->_multibody->getJointVel(_robot->_jointNameIndexList[i].first);
    }
}

void simMultiBodyDynamicsWorld::getRobotJointTorque(mMultiBody* _robot,Eigen::VectorXd& _q)
{
    if (_q.size() != _robot->_multibody->getNumDofs())
        throw std::runtime_error("vector size doesnot match with the no. of joints.Inside: getRobotJointTorque function\n");
    for (int i = 0; i < _robot->_multibody->getNumDofs(); i++)
    {
        _q[i] = _robot->_multibody->getJointTorque(_robot->_jointNameIndexList[i].first);
    }
}

void simMultiBodyDynamicsWorld::resetJointPos(mMultiBody* _robot, const Eigen::VectorXd& _q)
{
    if (_q.size() != _robot->_multibody->getNumDofs())
        throw std::runtime_error("vector size doesnot match with the no. of joints.Inside: resetJointPos function\n");
    
    int count = 0;
    for (auto it : _robot->_jointNameIndexList)
    {
        _robot->_multibody->setJointPos(it.first,_q[count]);
        ++count;
    }
    _robot->updateTransforms();
}

void simMultiBodyDynamicsWorld::setRobotJointTorque(mMultiBody* _robot,const Eigen::VectorXd& _q)
{
    if (_q.size() != _robot->_multibody->getNumDofs())
        throw std::runtime_error("vector size doesnot match with the no. of joints.Inside: setRobotJointTorque function\n");
    for (int i = 0; i < _robot->_multibody->getNumDofs(); i++)
    {
        _robot->_multibody->addJointTorque(_robot->_jointNameIndexList[i].first,btScalar(_q[i]));
    }
}

void simMultiBodyDynamicsWorld::printRobotJointsInfo(RobotObject* _robot)
{
    std::cout << "Printing only the joints which can be actuated." << std::endl;
    for (int i = 0; i < _robot->_jointNameIndexList.size(); i++)
    {
        std::cout << "Joint Name: " << _robot->_jointNameIndexList[i].second << " "
            << " Joint Index: " << _robot->_jointNameIndexList[i].first << std::endl;
    }
    
}

unsigned int simMultiBodyDynamicsWorld::addBodyBox(double l, double b, double h, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q)
{
    btCollisionShape* boxShape = new btBoxShape(btVector3(l/2,b/2,h/2));
    // save collision shapes.
    _rigidBodyCollisionShapes.push_back(boxShape);
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
        std::cout << "This Rigid body is Static. Type : Box "<< std::endl;
    else
    {
        boxShape->calculateLocalInertia(mass,localInertia);
    }
    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _rigidBodyMotionStates.push_back(myMotionState);
    // construct a rigid body.
    btRigidBody::btRigidBodyConstructionInfo rbinfo(mass,myMotionState,boxShape,localInertia);
    btRigidBody* body = new btRigidBody(rbinfo);
    _rigidBodyList.push_back(body);
    m_dynamicsWorld->addRigidBody(body);
    // if (isDynamics)
    //     m_dynamicsWorld->addRigidBody(body);
    // else
    //     m_dynamicsWorld->addRigidBody(body,1,1+2);
    body->setRestitution(0.5);
    return _rigidBodyList.size() - 1;
}

unsigned int simMultiBodyDynamicsWorld::addBodySphere(double r, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q)
{
    btCollisionShape* sphereShape = new btSphereShape(btScalar(r));
    // save collision shapes.
    _rigidBodyCollisionShapes.push_back(sphereShape);
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
        std::cout << "This Rigid body is Static. Type : Box "<< std::endl;
    else
    {
        sphereShape->calculateLocalInertia(mass,localInertia);
    }

    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _rigidBodyMotionStates.push_back(myMotionState);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sphereShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    body->setRestitution(0.5);

    m_dynamicsWorld->addRigidBody(body);
    _rigidBodyList.push_back(body);
     // return the index of the object.
    return _rigidBodyList.size() - 1 ;
}

unsigned int simMultiBodyDynamicsWorld::addBodyCylinder(double r, double h, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q)
{
    btCollisionShape* CylinderShape = new btCylinderShape(btVector3(r,h,1));
    // save collision shapes.
    _rigidBodyCollisionShapes.push_back(CylinderShape);
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
        std::cout << "This Rigid body is Static. Type : Box "<< std::endl;
    else
    {
        CylinderShape->calculateLocalInertia(mass,localInertia);
    }

    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _rigidBodyMotionStates.push_back(myMotionState);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, CylinderShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    body->setRestitution(0.5);

    m_dynamicsWorld->addRigidBody(body);
    _rigidBodyList.push_back(body);
     // return the index of the object.
    return _rigidBodyList.size() - 1 ;

}

unsigned int simMultiBodyDynamicsWorld::addBodyConvexHull(std::string _filename,double m,Eigen::Vector3d& _inertia, 
    Eigen::Vector3d _pose, Eigen::Quaterniond _q,Eigen::Vector3d _scale)
{
    btConvexHullShape* convexhullShape = new btConvexHullShape();
    // import mesh using assimp.
    Assimp::Importer Importer;
    const aiScene* pScene = Importer.ReadFile(_filename.c_str(), aiProcess_JoinIdenticalVertices);
    
    
    if (pScene != nullptr)
        std::cout << "num Meshes: " << pScene->mNumMeshes << std::endl;
    else
    {
        std::cerr << Importer.GetErrorString() << std::endl;
        throw std::runtime_error("pScene is null");
    }

    aiVector3D* ctr = pScene->mMeshes[0]->mVertices;
    // std::cout << pScene->mMeshes[0]->mNumVertices <<  std::endl;

    for (int i = 0; i < pScene->mNumMeshes; i++)
    {
        aiVector3D* _ctnr = pScene->mMeshes[i]->mVertices;
        // std::cout << pScene->mMeshes[i]->mNumVertices << std::endl;
        for (int j = 0; j < pScene->mMeshes[i]->mNumVertices; j++)
        {
            convexhullShape->addPoint(btVector3(_ctnr[j].x, _ctnr[j].y, _ctnr[j].z),false);
        }
        convexhullShape->recalcLocalAabb();
    }
    // convexhullShape->setMargin(0.01);
    // convexhullShape->optimizeConvexHull();
    btShapeHull shape(convexhullShape);
    shape.buildHull(convexhullShape->getMargin());
    // std::cout << shape.numVertices() << std::endl;

    btConvexHullShape* _convexhullshape;
    
    auto ptr = shape.getVertexPointer();        
    for (int i = 0; i < shape.numVertices(); i++)
    {
        // std::cout << ptr->x() << " " << ptr->y() << " " << ptr->z() << std::endl;
        _convexhullshape->addPoint(btVector3(ptr->x(),ptr->y(),ptr->z()),false);
        ptr++;
    }
    _convexhullshape->recalcLocalAabb();
    // delete old convex hull object.
    delete convexhullShape;
    // save the collision shape.
    _rigidBodyCollisionShapes.push_back(_convexhullshape);
     // setup transform.
     btTransform Transform;
     Transform.setIdentity();
     Transform.setOrigin(btVector3(_pose.x(),_pose.y(),_pose.z()));
     btQuaternion rot(_q.x(),_q.y(),_q.z(),_q.w());
     Transform.setRotation(rot);

     btScalar mass(m);
    btVector3 localInertia(_inertia.x(), _inertia.y(), _inertia.z());
    bool isDynamics = (mass != 0.f);
    if(!isDynamics)
        std::cout << "This Rigid body is Static. Type : ConvexHull filename: " << _filename << std::endl;
    else
    {
        _convexhullshape->calculateLocalInertia(mass,localInertia);
    }

    btDefaultMotionState* myMotionState = new btDefaultMotionState(Transform);
    _rigidBodyMotionStates.push_back(myMotionState);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, _convexhullshape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setRestitution(0.5);

    m_dynamicsWorld->addRigidBody(body);
    _rigidBodyList.push_back(body);
     // return the index of the object.
    return _rigidBodyList.size() - 1 ;   
}

void simMultiBodyDynamicsWorld::getBodyPoseAndRotation(unsigned int bodyIndex, Eigen::Vector3d& _pos,
                                                Eigen::Quaterniond& _q)
{
    btRigidBody* _rbody = _rigidBodyList[bodyIndex];
    btTransform trans;
    if (_rbody && _rbody->getMotionState())
    {
        _rbody->getMotionState()->getWorldTransform(trans);
    }

    _pos[0] = trans.getOrigin().getX();
    _pos[1] = trans.getOrigin().getY();
    _pos[2] = trans.getOrigin().getZ();

    // _q.w() = trans.getRotation().getW();
    // _q.x() = trans.getRotation().getX();
    // _q.y() = trans.getRotation().getY();
    // _q.z() = trans.getRotation().getZ();

    _q = Eigen::Quaterniond(trans.getRotation().getW(), trans.getRotation().getX(),
                            trans.getRotation().getY(), trans.getRotation().getZ());
}

// unsigned int simMultiBodyDynamicsWorld::attachForceSensorToRobot(std::string _robotName, std::string _linkName)
// {
//     bool _found = false;
//     RobotObject* _robot = _multibody_name_map[_robotName];
//     btMultiBody* p_multibody;
//     unsigned int _linkIndex;
//     for (int i = 0; i < _robot->_linkNameIndexList.size(); i++)
//     {
//         if ( _robot->_linkNameIndexList[i].second == _linkName)
//         {
//             _linkIndex = i;
//             _found = true;
//             break;
//         }
        
//     }
//     if (!_found)
//         std::runtime_error("Could find linkname in Robot object. _robotName: " + _robotName + " _linkName: "+ _linkName);
    
//     _force_sensors.push_back(std::pair<unsigned int, btMultiBodyJointFeedback*>(_force_sensors.size(),
//                                     _robot->_jointFeedbackIndexList[_linkIndex].second));
//     return _force_sensors.size() - 1;
// }

/**
 * @brief Can attach 1 force sensor(experimental).
*/
unsigned int simMultiBodyDynamicsWorld::attachForceSensorToRobot(RobotObject* _robot, unsigned int _ind, double _alpha)
{
    ForceSensor* ft_sensor = new ForceSensor(m_dynamicsWorld,_robot,_ind);
    if (_ft_sensors.size() == 0 && _ft_sensors.size() < 1)
        _ft_sensors.push_back(ft_sensor);
    else
        throw std::runtime_error("Cannot attach more than 1 force sensors.\n");
    // enable filter.
    ft_sensor->enableFilter(_alpha);
    return _ft_sensors.size();
}

void simMultiBodyDynamicsWorld::getForceSensorOutput(mMultiBody* _robotObject ,int _ind, Eigen::Vector3d& Force, Eigen::Vector3d& moment)
{
    RobotObject* _robot = _robotObject;
    btSpatialForceVector _f = _robot->_jointFeedbackIndexList.at(_ind).second->m_reactionForces;
    // btSpatialForceVector _g = _robot->_jointFeedbackIndexList.
    Force << _f.m_topVec.x(), _f.m_topVec.y(), _f.m_topVec.z();
    moment << _f.m_bottomVec.x(), _f.m_bottomVec.y(), _f.m_bottomVec.z(); 
}


void simMultiBodyDynamicsWorld::getForceSensorOutput(int _sensor_ind,Eigen::Vector3d& _force,
                                                            Eigen::Vector3d& _moment)
{
    ForceSensor* _ft_sensor = _ft_sensors.at(_sensor_ind);
    _ft_sensor->getForceMoment(_force,_moment);
}

void simMultiBodyDynamicsWorld::stepSimulation(float _ts, float _fixedStep)
{
    float _t_fixed;
    int _numSteps = 1;
    if (_ts <= _fixedStep)
    {
        _t_fixed = _ts;
        _numSteps = 1;
    }
    else
    {
        _numSteps = int(_ts/_fixedStep) + 1;
        _t_fixed = _fixedStep;
    }
    m_dynamicsWorld->stepSimulation(_ts,_numSteps,_t_fixed);
    
}

void simMultiBodyDynamicsWorld::convertStringTobtVector3(std::string _vstr, btVector3& _v, std::string _del)
{
   assert("The delimiter must be a one character array.\n" && _del.length() == 1);
   std::stringstream _ss(_vstr);
   std::string _substr;
   std::vector<double> _cntr;
   _cntr.resize(3); // as the btVector3 can have 3 elements.
   while (getline(_ss,_substr, _del[0]))
   {
        try
        {
           _cntr.push_back(std::stod(_substr));
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            throw std::runtime_error("Error while converting string to btVector3");
        }
   }

   _v.setX(_cntr[0]);
   _v.setY(_cntr[1]);
   _v.setZ(_cntr[2]);
      
}

void simMultiBodyDynamicsWorld::convertStringTobtQuaternion(std::string _qstr, btQuaternion& _q, std::string _del)
{
   assert("The delimiter must be a one character array.\n" && _del.length() == 1);
   std::stringstream _ss(_qstr);
   std::string _substr;
   std::vector<double> _cntr;
   _cntr.resize(3); // as the btVector3 can have 3 elements.
   while (getline(_ss,_substr, _del[0]))
   {
        try
        {
           _cntr.push_back(std::stod(_substr));
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            throw std::runtime_error("Error while converting string to btVector3");
        }
   }
   
   btQuaternion _qtemp;
   _qtemp.setEulerZYX(_cntr[0], _cntr[1], _cntr[2]);
   _q.setX(_qtemp.getX());
   _q.setY(_qtemp.getY());
   _q.setZ(_qtemp.getZ());
   _q.setW(_qtemp.getW());
}