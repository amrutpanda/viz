#include <simMultiBody.h>

simMultiBodyDynamicsWorld::simMultiBodyDynamicsWorld()
{
    //code.
    // _robot_list.resize(10);
    // _robot_mtbs.resize(10);
    InitialiseDynamicsWorld();
}

simMultiBodyDynamicsWorld::~simMultiBodyDynamicsWorld()
{
    delete m_solverinterface;

    if (_multibody_name_map.size() != 0)
    {
        for (auto it : _multibody_name_map)
        {
            delete it.second;
        }
    }

    if (_rigidbody_name_map.size() != 0)
    {
        for (auto it : _multibody_name_map)
        {
            delete it.second;
        }
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

void simMultiBodyDynamicsWorld::LoadRobotFromURDFFile(std::string _filename)
{
    BulletURDFImporter importer;
    if (!importer.ReadFile(_filename))
    {
        throw std::runtime_error("Error while reading file: " + _filename);
    }
    _multibody_name_map[importer.getName()] = importer.getMultiBodyStruct(m_dynamicsWorld);
    m_dynamicsWorld->addMultiBody(_multibody_name_map.at(importer.getName())->_multibody);
    // _multibody_name_map.at(importer.getName())->_multibody->setHasSelfCollision(false); // disable self-collision.
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
        for (auto l : mbs->_jointNameIndexMap)
        {
            std::cout << l.first << std::endl;
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
            assert((_q.size() == _mbs->_jointNameIndexMap.size(),"Mismatch in vector size to joint number. Inside getRobotJointPos\n"));
            break;
        }
        _count++;
    }

    btMultiBody* mb = _mbs->_multibody;
    for (int i = 0; i < _mbs->_jointNameIndexMap.size(); i++)
    {
        _q[i] = mb->getJointPos(i);
    } 
    
}

mMultiBody* simMultiBodyDynamicsWorld::getMultiBodyObject(int index)
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
    return _mbs;
}

int simMultiBodyDynamicsWorld::getNumRobots()
{
    return _multibody_name_map.size();
}

