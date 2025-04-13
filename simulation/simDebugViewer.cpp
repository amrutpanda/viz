#include <simDebugViewer.h>
#include <cstdlib>

bool runloop = true;

double randd() 
    {
        return (double)rand() / ((double)RAND_MAX + 1);
    }

void sighandler(int signum) {runloop = false;}

simDebugViewer::simDebugViewer(simMultiBodyDynamicsWorld* _world) : mGraphics("simDebugViewer")
{
    signal(SIGINT,sighandler);
    m_world = _world;
    m_world->setGravity(0,0,10);
    // m_world->InitialiseDynamicsWorld();  
    // mObject* s = new mObject();
    attachFlagVariable(&runloop);
    initApp();
    for ( int i = 0; i < m_world->getNumRobots(); i++)
    {
        // process_mMultibody(m_world->getMultiBodyObject(i));
        buildSimRobotGraphicsFromMultiBodyData(m_world->getMultiBodyObject(i));
        if (_show_collsion_shape)
        {
            _createCollisionMeshGraphicalObject(m_world->getMultiBodyObject(i),i);
        }     
        _mulitbody_objects.push_back(m_world->getMultiBodyObject(i));
    }
}

void simDebugViewer::process_mMultibody(mMultiBody* _robot)
{
    mObject* _aabbObj;
    mObject* _pObj;
    std::vector<mObject*> _mObj_vector;
    Ogre::SceneNode* _node;
    _robot_names.push_back(_robot->_name);
    btVector3 _pos;
    btQuaternion _q;
    for (int i = 0; i < _robot->_multibody->getNumLinks(); i++)
    {
        _aabbObj = new mObject();
        btMultibodyLink _mtb_link = _robot->_multibody->getLink(i);
        int pind = _robot->_multibody->getLink(i).m_parent;
        // std::cout << i << " " << pind << std::endl;
        btMultibodyLink _pmtb_link = _robot->_multibody->getLink(pind);
        if (i == 0)
        {
            _node = scnMgr->getRootSceneNode()->createChildSceneNode();
            _aabbObj->setSceneNode(_node);
            _pObj = _aabbObj;
            _pos.setZero();
            _q = btQuaternion(0,0,0,1);
        }
        else
        {
            _pObj = _mObj_vector[pind];
            _node = _pObj->getSceneNode()->createChildSceneNode();
            _aabbObj->setSceneNode(_node);
            _pos = _pmtb_link.m_dVector + _mtb_link.m_eVector;
            // _pos = _pmtb_link.m_dVector + _mtb_link.m_cachedRVector;
            _q = _mtb_link.m_zeroRotParentToThis;
            // _q = _robot->_multibody->getLink(i
            
        }
        // printVector(_pos,"pose");
        // printQuaternion(_q,"_q");
        // attachAabb(_mtb_link,_aabbObj);
        _aabbObj->setPosition(Ogre::Vector3(_pos.x(), _pos.y(), _pos.z()));
        _aabbObj->setRotation(Ogre::Quaternion(_q.w(),_q.x(),_q.y(),_q.z()));
        _aabbObj->setAxis();
        _aabbObj->setAxisVisible(true); 
        _mObj_vector.push_back(_aabbObj);
        
        // if (i == 0)
        // {
        //     break;
        // }
        
    }
    _robot_objects.push_back(_mObj_vector);
}

void simDebugViewer::buildSimRobotGraphicsFromMultiBodyData(mMultiBody* _robot)
{
    mObject* _aabbObj;
    Ogre::SceneNode* _node;
    std::vector<mObject*> _mObj_vector;
    _robot_names.push_back(_robot->_name);
    // create graphical objects.
    for (int i = 0; i < _robot->_multibody->getNumLinks()+1 ; i++)
    {
        _node = scnMgr->getRootSceneNode()->createChildSceneNode();
        _aabbObj = new mObject();
        _aabbObj->setSceneNode(_node);
        _mObj_vector.push_back(_aabbObj);
    }
    _robot_objects.push_back(_mObj_vector);
    // compute forward kinematics;
    _robot->_multibody->forwardKinematics(rot_world_to_local, local_origin_world_frame); // coordinate of link com in world frame.
    std::cout << _robot->_multibody->getNumLinks()+1 << std::endl;
    for (int i = 0; i < rot_world_to_local.size(); i++)
    {
        _aabbObj = _mObj_vector[i];
        _aabbObj->setPosition(Ogre::Vector3(local_origin_world_frame[i].x(), local_origin_world_frame[i].y(), local_origin_world_frame[i].z()));
        _aabbObj->setRotation(Ogre::Quaternion(Ogre::Quaternion(rot_world_to_local[i].w(), rot_world_to_local[i].x(),rot_world_to_local[i].y(), rot_world_to_local[i].z())));
        _aabbObj->setAxis();
        _aabbObj->setAxisVisible(true);
        // printVector(local_origin_world_frame[i],"pose");
        // btScalar r,p,y;
        // rot_world_to_local[i].getEulerZYX(y,p,r);
        // std::cout << r << " " << p << " " << y << std::endl;
        // attachAabb(_robot->_multibody->getLink(i),_aabbObj);
        
    }
}

void simDebugViewer::updateSimRobotGraphics(int _robot_index)
{
    mObject* _ptr;
    std::vector<mObject*> _mObjList = _robot_objects[_robot_index];

    mMultiBody* _mtObj = _mulitbody_objects[_robot_index];
    btVector3 _p = _mtObj->_multibody->getBasePos();
    btQuaternion _q = _mtObj->_multibody->getWorldToBaseRot();
    _mtObj->updateTransforms();
    for (int i = 0; i < _mtObj->_multibody->getNumLinks()+1; i++)
    {
        _ptr = _mObjList[i];
        if (i != 0)
        {   
            _p = _mtObj->_multibody->getLink(i-1).m_cachedWorldTransform.getOrigin();
            _q = _mtObj->_multibody->getLink(i-1).m_cachedWorldTransform.getRotation(); // need to check it.
        }
        _ptr->setPosition(Ogre::Vector3(_p.x(),_p.y(), _p.z()));
        _ptr->setRotation(Ogre::Quaternion(_q.w(), _q.x(), _q.y(), _q.z()));
    }
    
}

void simDebugViewer::attachAabb(btMultibodyLink& _link, mObject* _mObj, btVector3 _color) // to-do: resolve bug.
{
    btVector3 aabbMin, aabbMax, l;
    aabbMin.setZero();
    aabbMax.setZero();
    btTransform _tr;
    _tr.setIdentity();
    
    // _link.m_collider->getCollisionShape()->calculateTemporalAabb()
    if (_link.m_collider != nullptr)
    {
        if (_link.m_collider->getCollisionShape() != nullptr)
        {
            _link.m_collider->getCollisionShape()->getAabb(_tr, aabbMin,aabbMax);
        }
    }

    printVector(aabbMin,"aabbMin");
    printVector(aabbMax,"aabbMax");
    
    l = aabbMax - aabbMin;
    std::vector<std::pair<btVector3,btVector3>> box_points;
    std::pair<btVector3,btVector3> _line;
    // top rectangle.

    _line.first = aabbMax; 
    _line.second = aabbMax - btVector3(l[0],0,0);
    box_points.push_back(_line);

    _line.first = aabbMax - btVector3(l[0], 0, 0);
    _line.second = aabbMax - btVector3(l[0],l[1],0);
    box_points.push_back(_line);

    _line.first = aabbMax - btVector3(l[0], l[1], 0);
    _line.second = aabbMax - btVector3(0, l[1], 0);
    box_points.push_back(_line);

    _line.first = aabbMax - btVector3(0, l[1], 0);
    _line.second = aabbMax;
    box_points.push_back(_line);

    // bottom rectangle.

    _line.first = aabbMin;
    _line.second = aabbMin + btVector3(l[0], 0 , 0);
    box_points.push_back(_line);

    _line.first = aabbMin + btVector3(l[0], 0 , 0);
    _line.second = aabbMin + btVector3(l[0], l[1] , 0);
    box_points.push_back(_line);

    _line.first = aabbMin + btVector3(l[0], l[1] , 0);
    _line.second = aabbMin +  btVector3(0, l[1] , 0);
    box_points.push_back(_line);

    _line.first = aabbMin + btVector3(0, l[1] , 0);
    _line.second = aabbMin;
    box_points.push_back(_line);

    // four sides connecting top rectangle to bottom rectangle.

    _line.first = aabbMax;
    _line.second = aabbMin + btVector3(l[0], l[1] , 0);
    box_points.push_back(_line);

    _line.first = aabbMin;
    _line.second = aabbMax - btVector3(l[0], l[1] , 0);
    box_points.push_back(_line);

    _line.first = aabbMax - btVector3(0, l[1] , 0);
    _line.second = aabbMin + btVector3(l[0], 0 , 0);
    box_points.push_back(_line);

    _line.first = aabbMax - btVector3(l[0], 0 , 0);;
    _line.second = aabbMin + btVector3(0, l[1] , 0);
    box_points.push_back(_line);

    // finished computing points.

    // create manual objects.
    _count++;
    std::string _meshName = "_box"+std::to_string(_count);
    Ogre::ManualObject* man = scnMgr->createManualObject("_box"+std::to_string(_count));
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("Line","UserData");
    mat->getTechnique(0)->getPass(0)->setLineWidth(2);
    mat->getTechnique(0)->getPass(0)->setAmbient(_color.x(),_color.y(),_color.z());
    mat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(_color.x(),_color.y(),_color.z()));

    man->begin(mat,Ogre::RenderOperation::OT_LINE_LIST);
    for (int i = 0; i < box_points.size(); i++)
    {
        man->position(Ogre::Vector3(box_points[i].first.x(), box_points[i].first.y(), box_points[i].first.z()));
        man->colour(Ogre::ColourValue(_color.x(),_color.y(),_color.z()));
        man->position(Ogre::Vector3(box_points[i].second.x(), box_points[i].second.y(), box_points[i].second.z()));
        man->colour(Ogre::ColourValue(_color.x(),_color.y(),_color.z()));
    }
    man->end();
    Ogre::MeshPtr mptr = man->convertToMesh(_meshName,"UserData");

    // Ogre::SceneNode* _node = _mObj->getSceneNode();
    // _node->attachObject(man);

    _mObj->attachChildMesh(scnMgr,_meshName,Ogre::Vector3(0),Ogre::Quaternion(1,0,0,0));
    _mObj->attachChildMesh(scnMgr,"mSphere.mesh",Ogre::Vector3(0),Ogre::Quaternion(1,0,0,0), Ogre::Vector3(0.02));

}

void simDebugViewer::_createCollisionMeshGraphicalObject(mMultiBody* _robot, int _robot_index)
{
    // create separate graphical objects to visualize collision.
    Ogre::SceneNode* _node;
    mObject* _mObj;
    std::vector<mObject*> _colmObj_list;
    for (int i = 0; i < _robot->_colliders.size(); i++)
    {
        _node = scnMgr->getRootSceneNode()->createChildSceneNode();
        _mObj = new mObject();
        _mObj->setSceneNode(_node);
        // _mObj->setAxis();
        // _mObj->setAxisVisible(true);
        _colmObj_list.push_back(_mObj);
    }
    // store it.
    _robot_collision_shapes_objects.push_back(_colmObj_list);
    rot_world_to_local.resize(0);
    local_origin_world_frame.resize(0);
    _robot->updateTransforms();
    // _robot->_multibody->updateCollisionObjectWorldTransforms(rot_world_to_local,local_origin_world_frame);
    btMultiBody* p_multibody = _robot->_multibody;

    for (int i = 1; i < _robot->_colliders.size(); i++)
    {
        std::string _mesh_name;
        btMultiBodyLinkCollider* _col = _robot->_colliders[i];

        if (_col == nullptr)
            continue;
        btCollisionShape* _shape = _col->getCollisionShape();
        if (_shape == nullptr)
            continue;
        
        // mObject* _ptr = _colmObj_list[i];
        mObject* _ptr = _robot_objects[_robot_index][i];
        btConvexHullShape* _chshape = dynamic_cast<btConvexHullShape*>(_shape);
        btCompoundShape* _cmpd_shape = dynamic_cast<btCompoundShape*>(_shape);
        std::cout << "i = " << i << std::endl;
        btTransform _link_com_tr =  _robot->_multibody->getLink(i).m_cachedWorldTransform;
        if (_chshape != nullptr)
        {
            std::cout << "Got convex mesh\n";
            _processConvexHullShape(_chshape,_ptr,_mesh_name);

            btVector3 _scale = _chshape->getLocalScaling();
            if (_col->getUserPointer() != nullptr)
            {
                btTransform* _tr = static_cast<btTransform*>(_col->getUserPointer());
                btVector3 _p = _tr->getOrigin();
                btQuaternion _q = _tr->getRotation();
                
                // btQuaternion _q = _col->getWorldTransform().getRotation();
                printVector(_p,"pos");
                _ptr->attachChildMesh(scnMgr,_mesh_name,Ogre::Vector3(_p.x(), _p.y(), _p.z()),
                                                        Ogre::Quaternion(_q.w(),_q.x(),_q.y(), _q.z()),
                                                        Ogre::Vector3(_scale.x(), _scale.y(), _scale.z()));
                // std::cout << "_child pos: "<< _ptr->getChildMeshNode(_mesh_name)->_getDerivedPosition() << std::endl;
                continue;
            }

            // btVector3 _p = _col->getWorldTransform().getOrigin();


            _ptr->attachChildMesh(scnMgr,_mesh_name,Ogre::Vector3(0, 0, 0),
                                                    Ogre::Quaternion(1 ,0 ,0 ,0 ),
                                                    Ogre::Vector3(_scale.x(), _scale.y(), _scale.z()));
            // _ptr->setPosition(Ogre::Vector3(_p.x(), _p.y(), _p.z()));
            // _ptr->setRotation(Ogre::Quaternion(_q.w(),_q.x(), _q.y(), _q.z()));
        }
        else if (_cmpd_shape != nullptr)
        {
            std::cout << "Got compound mesh\n";
            // btTransform _link_com_tr =  _robot->_multibody->getLink(i).m_cachedWorldTransform;
            for (int j = 0; j < _cmpd_shape->getNumChildShapes(); j++)
            {
                _chshape = dynamic_cast<btConvexHullShape*>(_cmpd_shape->getChildShape(j));
                btTransform _ch_tr = _cmpd_shape->getChildTransform(j);
                if (_chshape != nullptr )
                {
                    _processConvexHullShape(_chshape,_ptr,_mesh_name);
                    btVector3 _p = _ch_tr.getOrigin() - _link_com_tr.getOrigin();
                    btQuaternion _q = _ch_tr.getRotation();
                    btVector3 _scale = _chshape->getLocalScaling();
                    printVector(_scale,"scale");
                    // _ptr->attachChildMesh(scnMgr,_mesh_name,Ogre::Vector3(_p.x(), _p.y(), _p.z()),
                    //                                     Ogre::Quaternion(_q.w(),_q.x(),_q.y(), _q.z()),
                    //                                     Ogre::Vector3(_scale.x(),_scale.y(), _scale.z()));

                    // _ptr->attachChildMesh(scnMgr,_mesh_name,Ogre::Vector3(0),
                    //                                 Ogre::Quaternion(0,0,0, 1),
                    //                                 Ogre::Vector3(_scale.x(),_scale.y(), _scale.z()));
                    
                }
            }
            
        }
    }
    std::cout << "Finished >>>\n";
}

void simDebugViewer::_processConvexHullShape(btConvexHullShape* _shape, mObject* _mObj, std::string& _mesh_name)
{
    btShapeHull* shape_mesh = static_cast<btShapeHull*>(_shape->getUserPointer());
    if (shape_mesh == nullptr)
        throw std::runtime_error("No shapehull pointer assigned.(inside _processConvexHullShape function.\n");
    
    Ogre::ManualObject* man = scnMgr->createManualObject();
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("Collision","UserData");
    // mat->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(randd(), randd(), randd()));
    // std::cout << "Color value: " << mat->getTechnique(0)->getPass(0)->getAmbient() << std::endl;
    man->begin(mat,Ogre::RenderOperation::OT_TRIANGLE_LIST);
    btVector3* _ptr = const_cast<btVector3*> (shape_mesh->getVertexPointer());
    unsigned int* _idx = const_cast<unsigned int*>(shape_mesh->getIndexPointer());
    int count = 0;
    std::cout << "Extracting points from shapehull\n";
    for (int i = 0; i < shape_mesh->numTriangles(); i++)
    {
        man->position(Ogre::Vector3(_ptr->x(),_ptr->y(),_ptr->z()));
        man->colour(Ogre::ColourValue(randd(), randd(), randd()));
        _ptr++;
        man->position(Ogre::Vector3(_ptr->x(),_ptr->y(),_ptr->z()));
        man->colour(Ogre::ColourValue(randd(), randd(), randd()));
        _ptr++;
        man->position(Ogre::Vector3(_ptr->x(),_ptr->y(),_ptr->z()));
        man->colour(Ogre::ColourValue(randd(), randd(), randd()));
        _ptr++;
        man->index(*_idx);
        _idx++;
        man->index(*_idx);
        _idx++;
        man->index(*_idx);
        _idx++;
    }
    man->end();
    std::cout << "Filled manualobject with points from shapehull\n";
    _mesh_name.clear();
    _count++;
    _mesh_name = "mesh_" + std::to_string(_count);
    Ogre::MeshPtr _mptr = man->convertToMesh(_mesh_name,"UserData");
    std::cout << "Finished converting to mesh\n";
    
}

void simDebugViewer::renderViewer()
{
    // for (int i = 0; i < 500; i++)
    // {
    //     RenderOneFrame();
    // }
    
    m_world->getMultiBodyObject(0)->_multibody->addJointTorque(0,-0.11);
    while (runloop)
    {
        RenderOneFrame();
        updateSimRobotGraphics(0);
        m_world->getMultiBodyObject(0)->_multibody->addJointTorque(7,-01.01);
        // std::cout << m_world->getMultiBodyObject(0)->_multibody->getJointPos(0) << std::endl;
        // std::cout << m_world->getMultiBodyObject(0)->_multibody->getJointTorque(3) << std::endl;
        // m_world->getMultiBodyObject(0)->_multibody->addLinkTorque(3,btVector3(10,10,0));
        // std::cout <<   m_world->getMultiBodyObject(0)->_multibody->getInterpolateRVector(3).z() << std::endl;
        m_world->m_dynamicsWorld->stepSimulation(0.002,4,0.001);
        // sleep(0.2);
        // std::cout << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        // m_world->getMultiBodyObject(0)->_multibody->addBaseTorque(btVector3(10,10,10));
    }
    
}

simDebugViewer::~simDebugViewer()
{
}