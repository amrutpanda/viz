#include <BulletURDFImporter.h>

void printVector(btVector3& _v, std::string _str)
{
    std::cout << _str << ": " << _v.x() <<  " " << _v.y() << " " << _v.z() << std::endl;
}
void printQuaternion(btQuaternion& _v, std::string _str)
{
    if (_str.empty())
        _str = "Rot";
        std::cout << _str << ": " << _v.x() <<  " " << _v.y() << " " << _v.z() << " " << _v.w() << std::endl;
}


bool BulletURDFImporter::ReadFile(std::string _filename, bool _fixedBase)
{
    _urdf = urdf::parseURDFFile(_filename);
    // Process path.
    std::filesystem::path path = std::filesystem::path(_filename);
    // URDF_PATH = path.parent_path();
    URDF_PATH = path;
    std::cout << "urdf root path : " << URDF_PATH << std::endl;

    // set up base pose and orientation;
    _base_pos = btVector3(0,0,0);
    _base_rot = btQuaternion(0,0,0,1);
    // get urdf root;
    urdf::LinkConstSharedPtr _r = _urdf->getRoot();
    fixedBase = _fixedBase;
    numLinks = _urdf->links_.size();
    numJoints = _urdf->joints_.size();
    // store robot name.
    m_name = _urdf->getName();
    
    urdf::Link* _rootLink = const_cast<urdf::Link*>(_r.get());
    baseLink = _rootLink;

    std::cout << "Parsing urdf \n";
    ParseURDFTree(_rootLink);
    std::cout << "Parsing complete\n";
    return true;
}

void BulletURDFImporter::ParseURDFTree(urdf::Link* _rootLink)
{
    std::vector<urdf::Link*> _tptr, _pptr, _cptr;
    std::vector<int> p_ind, c_ind, t_ind;

    int last_ind = 0, child_ind;
    p_ind.push_back(last_ind);
    last_ind++;
    _pptr.push_back(_rootLink);

    // create btMultiBody object and set base of the multibody it.
    createMultiBody(_rootLink);
    // create multibody from base link.
    createMultiBodyCompFromURDFLink(0,-1,_rootLink);

    urdf::Link* _ulink;
    urdf::Joint* _ujoint;
   
    while (!_pptr.empty())
    {
        // std::cout << _pptr.size() << std::endl;
        for (int i = 0; i < _pptr.size(); i++)
        {
            // std::cout << _pptr[i]->name << " "; 
            for (int j = 0; j < _pptr[i]->child_links.size(); j++)
            {
                _ulink = _pptr[i]->child_links[j].get();
                _cptr.push_back(_ulink);

                c_ind.push_back(last_ind);
                child_ind = last_ind;
                //
                // add code to create a rigidbody in multibody tree here.
                createMultiBodyCompFromURDFLink(child_ind,p_ind[i],_ulink);
                //
                std::cout << "( " << p_ind[i] <<" " << last_ind << " )" << "\n";
                last_ind++;

            }
            // std::cout << "    ";
            // std::cout << "++++++++++++++++++" << std::endl;
        }
        // std::cout << "\n";
        // std::cout << _cptr.size() << std::endl;
        // std::cout << "++++++++++++++++++" << std::endl;
        // swap the vectors.
        _tptr = _cptr;
        _cptr = _pptr;
        _pptr = _tptr;

        // swap ind vectors.
        t_ind = c_ind;
        c_ind = p_ind;
        p_ind = t_ind;

        // clear the vector containing children after making it pptr.
        _cptr.clear();
        c_ind.clear();
         
    }
    // finaliza the multibody.
    p_multibody->finalizeMultiDof();
    // std::cout << p_multibody->getNumDofs() << " " << p_multibody->getNumLinks() << " " << numLinks << std::endl;
    // std::cout << m_multibody->_name << std::endl;
    // std::cout << "All values printed\n";
}

void BulletURDFImporter::createMultiBody(urdf::Link* _baseLink)
{
    btScalar mass = 0;
    btVector3 InertiaMatrix(0,0,0);

    if (_baseLink->inertial.get() != nullptr)
    {
        urdf::InertialSharedPtr iptr = _baseLink->inertial;
        mass = iptr->mass;
        InertiaMatrix = btVector3(iptr->ixx,iptr->iyy,iptr->izz);
    }

    // create a mMultiBody struct and store data into it.
    m_multibody = new mMultiBody();
    m_multibody->_name = m_name;
    m_multibody->_multibody = new btMultiBody(numLinks,mass,InertiaMatrix,fixedBase,false);
    p_multibody = m_multibody->_multibody;
    p_multibody->setBasePos(_base_pos);
    p_multibody->setWorldToBaseRot(_base_rot);
    
}

void BulletURDFImporter::addLinkToMultiBody(urdf::Link* ulink, mMultiBody* _pMultiBody)
{
    urdf::CollisionSharedPtr _colptr;
    p_multibody = m_multibody->_multibody;
    btScalar linkmass(0);
    btVector3 linkInertiaDiag(0,0,0);

    // p_multibody->setupRevolute()
}

void BulletURDFImporter::createMultiBodyCompFromURDFLink(int linkIndex, int parentIndex, urdf::Link* _link, float _mass)
{
    urdf::JointSharedPtr _pJoint = _link->parent_joint;
    btScalar mass = _mass;
    btVector3 Inertia(0,0,0);

    std::cout << _link->name << std::endl;
    if (_link->inertial.get() != nullptr)
    {
        urdf::InertialSharedPtr iptr = _link->inertial;
        mass = iptr->mass;
        // TO-DO: actual evaluation of Inertia tensor to diagonal vector will be implemented.
        
        // btMatrix3x3 inr_mat;
        // inr_mat[0][0] = iptr->ixx;
        // inr_mat[0][1] = iptr->ixy;
        // inr_mat[0][2] = iptr->ixz;
        // inr_mat[1][0] = inr_mat[0][1];
        // inr_mat[1][1] = iptr->iyy;
        // inr_mat[1][2] = iptr->iyz;
        // inr_mat[2][0] = inr_mat[0][2];
        // inr_mat[2][1] = inr_mat[1][2];
        // inr_mat[2][2] = iptr->izz;
        // convertStringToInertiaVector(inr_mat,Inertia);

        Inertia = btVector3(iptr->ixx,iptr->iyy,iptr->izz);
    }
    btVector3 parentOriginToThisLinkOffset(0,0,0),_joint_rotation_axis(0,0,0);// axis needed as an argument.
    btVector3 parentComToThisPivotOffset(0,0,0), thisLinkPivotToThisLinkComOffset(0,0,0); // needed as argument
    btQuaternion rotParentToThisLink(0,0,0,1); // needed as argument
    btVector3 parentOriginToComOffset = btVector3(0,0,0);
    
    if (_pJoint.get() != nullptr)
    {
        
        parentOriginToThisLinkOffset = btVector3(_pJoint->parent_to_joint_origin_transform.position.x,
                                            _pJoint->parent_to_joint_origin_transform.position.y,
                                            _pJoint->parent_to_joint_origin_transform.position.z );
        urdf::Rotation _rotation = _pJoint->parent_to_joint_origin_transform.rotation.GetInverse(); // it is parent points rotated in current link
                                                                                                    // frame, so the inverse.
        // rotParentToThisLink = btQuaternion(_pJoint->parent_to_joint_origin_transform.rotation.x,
        //                                     _pJoint->parent_to_joint_origin_transform.rotation.y,
        //                                     _pJoint->parent_to_joint_origin_transform.rotation.z,
        //                                     _pJoint->parent_to_joint_origin_transform.rotation.w);
        rotParentToThisLink = btQuaternion(_rotation.x, _rotation.y,_rotation.z,_rotation.w);
        _joint_rotation_axis = btVector3(_pJoint->axis.x, _pJoint->axis.y, _pJoint->axis.z);
       
        if (_link->getParent()->inertial.get() != nullptr)
        {
            parentOriginToComOffset = btVector3(_link->getParent()->inertial->origin.position.x,
                                                _link->getParent()->inertial->origin.position.y,
                                                _link->getParent()->inertial->origin.position.z);
            
        }    
    }

    if (_link->inertial.get() != nullptr)
    {
        thisLinkPivotToThisLinkComOffset = btVector3(_link->inertial->origin.position.x,
                                                     _link->inertial->origin.position.y,
                                                     _link->inertial->origin.position.z);
    }

    parentComToThisPivotOffset = parentOriginToThisLinkOffset - parentOriginToComOffset;
    
    printVector(parentOriginToThisLinkOffset,"parentOriginToThisLinkOffset");
    printVector(parentOriginToComOffset,"parentOriginToComOffset");
    printVector(parentComToThisPivotOffset,"parentComToThisPivotOffset");
    printVector(thisLinkPivotToThisLinkComOffset,"thisLinkPivotToThisLinkComOffset");
    printQuaternion(rotParentToThisLink,"rotParentToThisLink");

    // create a multibody rigidbody.
    if (_pJoint.get() != nullptr)
    {
        std::cout << "joint name: " << _pJoint->name << " " << _pJoint->type << std::endl;
       
        switch (_pJoint->type)
        {
        case urdf::Joint::FIXED :
            {
                p_multibody->setupFixed(linkIndex,mass,Inertia,parentIndex,rotParentToThisLink,
                                            parentComToThisPivotOffset, thisLinkPivotToThisLinkComOffset);
                m_multibody->_fixedJointNameIndexList.push_back(std::pair<int,std::string>(linkIndex,_link->name));
                break;
            }
        case urdf::Joint::REVOLUTE :
            {
                p_multibody->setupRevolute(linkIndex,mass,Inertia,parentIndex,rotParentToThisLink,_joint_rotation_axis,
                                                parentComToThisPivotOffset,thisLinkPivotToThisLinkComOffset);
                // m_multibody->_jointNameIndexMap[_pJoint->name] = linkIndex;
                m_multibody->_jointNameIndexList.push_back(std::pair<int,std::string>(linkIndex,_pJoint->name));
        
                break;
            }
        case urdf::Joint::CONTINUOUS :
            {
                p_multibody->setupRevolute(linkIndex,mass,Inertia,parentIndex,rotParentToThisLink,_joint_rotation_axis,
                                                    parentComToThisPivotOffset,thisLinkPivotToThisLinkComOffset);
                // m_multibody->_jointNameIndexMap[_pJoint->name] = linkIndex;
                m_multibody->_jointNameIndexList.push_back(std::pair<int,std::string>(linkIndex,_pJoint->name));
                break;
            }
        case urdf::Joint::PRISMATIC :
            {
                // disableParentCollision has been set to false.
                p_multibody->setupPrismatic(linkIndex,mass,Inertia,parentIndex,rotParentToThisLink,_joint_rotation_axis,
                                                parentComToThisPivotOffset,thisLinkPivotToThisLinkComOffset,true);
                // m_multibody->_jointNameIndexMap[_pJoint->name] = linkIndex;
                m_multibody->_jointNameIndexList.push_back(std::pair<int,std::string>(linkIndex,_pJoint->name));
                break;
            }

        default:
            break;
        }
    }
    else
    {
        p_multibody->setupFixed(linkIndex,mass,Inertia,parentIndex,rotParentToThisLink,
            parentComToThisPivotOffset, thisLinkPivotToThisLinkComOffset);
    }

    p_multibody->getLink(linkIndex).m_linkName = _link->name.c_str();
    std::cout << _link->name << " " << p_multibody->getLink(linkIndex).m_linkName << std::endl;
    // m_multibody->_linkNameIndexMap[_link->name] = linkIndex;
    m_multibody->_linkNameIndexList.push_back(std::pair<int, std::string>(linkIndex,_link->name));
    // create collision shapes.

}

bool BulletURDFImporter::createMultiBodyLinkCollisionShapes(int linkIndex, urdf::Link* _link)
{
    std::cout << "Processing collision mesh for link: " << _link->name << std::endl;
    if (_link->collision_array.size() == 0)
    {
        m_multibody->_colShapes.push_back(nullptr);
        m_multibody->_colliders.push_back(nullptr);
        return false;
    }
        // return false;
        
    btCompoundShape* _cmpd_shape;
    if (_link->collision_array.size() > 1)
        _cmpd_shape = new btCompoundShape(_link->collision_array.size());
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(p_multibody,linkIndex);
    btCollisionShape* shape;
    
    for (int i = 0; i < _link->collision_array.size(); i++)
    {
        urdf::CollisionSharedPtr cptr = _link->collision_array[i];
        int type = cptr->geometry->type;
        // btCollisionShape* shape;
        
        btVector3 _pos(cptr->origin.position.x, cptr->origin.position.y, cptr->origin.position.z);
        btQuaternion _rot(cptr->origin.rotation.x, cptr->origin.rotation.y, cptr->origin.rotation.z, cptr->origin.rotation.w);

        switch (type)
        {
        case urdf::Geometry::BOX :
            {
                urdf::Box* boxptr = dynamic_cast<urdf::Box*>(cptr->geometry.get());
                btVector3 dim(boxptr->dim.x/2, boxptr->dim.y/2,boxptr->dim.z/2); // TO-DO: need to verify the box size.
                // btVector3 dim(boxptr->dim.x, boxptr->dim.y,boxptr->dim.z);
                shape = new btBoxShape(dim);
                shape->setMargin(0);
                break;
            }
        case urdf::Geometry::CYLINDER :
            {
                urdf::Cylinder* cldptr = dynamic_cast<urdf::Cylinder*>(cptr->geometry.get());
                // btVector3 dim(cldptr->radius, cldptr->length, 1); // 1 stands for Z axis direction
                btVector3 dim(cldptr->radius, cldptr->length/10, cldptr->radius);
                // printVector(dim,"Cylinder dims: ");
                shape = new btCylinderShape(dim);
                // shape = new btCylinderShapeZ(dim);
                shape->setMargin(0);
                break;
            }
        case urdf::Geometry::SPHERE :
            {
                urdf::Sphere* sptr = dynamic_cast<urdf::Sphere*>(cptr->geometry.get());
                shape = new btSphereShape(btScalar(sptr->radius));
                shape->setMargin(0);
                break;
            }
        case urdf::Geometry::MESH :
            {
                urdf::Mesh* mptr = dynamic_cast<urdf::Mesh*>(cptr->geometry.get());
                std::filesystem::path path = std::filesystem::path(mptr->filename);
                
                std::string _filename = resolvePath(mptr->filename);
                // std::string _filename = mptr->filename;
                std::cout << "found mesh: " << _filename << std::endl;                
                // break;
                
                btConvexHullShape* chshape = new btConvexHullShape();
                btConvexHullShape* tshape = new btConvexHullShape();

                Assimp::Importer importer;
                const aiScene* pScene = importer.ReadFile(_filename.c_str(), aiProcess_JoinIdenticalVertices);

                if (pScene != nullptr)
                    std::cout << "num Meshes: " << pScene->mNumMeshes << std::endl;
                else
                {
                    std::cerr << importer.GetErrorString() << std::endl;
                    throw std::runtime_error("pScene is null");
                }

                for (int i = 0; i < pScene->mNumMeshes; i++)
                {
                    aiVector3D* _ctnr = pScene->mMeshes[i]->mVertices;
                    for (int j = 0; j < pScene->mMeshes[i]->mNumVertices; j++)
                    {
                        // std::cout << ctr[j].x << " " << ctr[j].y << " " << ctr[j].z << " " << std::endl;
                        tshape->addPoint(btVector3(_ctnr[j].x, _ctnr[j].y, _ctnr[j].z),false);
                    }
                    tshape->recalcLocalAabb();
                }
                // experimental: set margin to be zero.
                tshape->setMargin(0);
                // create a shapehull builder.
                btShapeHull* hull = new btShapeHull(tshape);
                std::cout << "Margin: " << tshape->getMargin() << std::endl;
                std::cout << "keeping the margin to be zero." << std::endl;
                hull->buildHull(0*tshape->getMargin());
                auto ptr = hull->getVertexPointer();       

                for (int i = 0; i < hull->numVertices(); i++)
                {
                    // std::cout << ptr->x() << " " << ptr->y() << " " << ptr->z() << std::endl;
                    chshape->addPoint(btVector3(ptr->x(),ptr->y(),ptr->z()));
                    ptr++;
                }
                // // deallocate the tshape object.
                delete tshape;
                shape = chshape;
                // store the hull pointer for future usage for visualization purpose.
                chshape->setUserPointer(hull);
                break;
            }
        default:
            {
                std::cout << "Cannot process Collision information. Skipping ..." << std::endl;
            }
            break;
        }
        
        // btTransform tr;
        // if (!_find_collision_mesh_world_transform(col, linkIndex,_pos,_rot,tr));
        //     col->setWorldTransform(tr); // will be executed more than once in case of multiple mesh,but benign.

        if (_link->collision_array.size() > 1)
        {
            btTransform _local_tr;
            _find_collision_mesh_local_transform(col, linkIndex,_pos,_rot,_local_tr);
            _cmpd_shape->addChildShape(_local_tr,shape);
        
        }
        else
        {
            btTransform tr;
            if (!_find_collision_mesh_world_transform(col, linkIndex,_pos,_rot,tr));
                col->setWorldTransform(tr); 
            col->setCollisionShape(shape);
        }
        
    }
    // add compoundshape to the link collider object.
    if (_link->collision_array.size() > 1)
    {
        std::cout << "Multiple collision meshes detected" << std::endl;
        // change the transform to identity.
        col->setUserPointer(new btTransform);
        static_cast<btTransform*>(col->getUserPointer())->setIdentity();
        col->setCollisionShape(_cmpd_shape);
        p_multibody->getLink(linkIndex).m_collider = col;
        m_multibody->_colShapes.push_back(_cmpd_shape);
        m_multibody->_colliders.push_back(col);
    }
    else
    {
        p_multibody->getLink(linkIndex).m_collider = col;
        m_multibody->_colShapes.push_back(shape);
        m_multibody->_colliders.push_back(col);
    }
    // setting up collsion flags for the collider.

    // col->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    // setup collsion group and mask.

    bool isDynamic = (p_multibody->getLinkMass(linkIndex) == 0) ? false : true;
    int collisionfiltergroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
    int collisionfiltermask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
    m_world->addCollisionObject(col,collisionfiltergroup, collisionfiltermask); // TO-DO: Need to evaluate other two arguments which are filter masks.
    // m_world->addCollisionObject(col);
    
    if (p_multibody->getLink(linkIndex).m_jointFeedback == nullptr)
    {
        std::cout << "No jointfeedback pointer set. Setting now..\n";
        btMultiBodyJointFeedback* _jointfb = new btMultiBodyJointFeedback();
        p_multibody->getLink(linkIndex).m_jointFeedback = _jointfb;
        m_multibody->_jointFeedbackIndexList.push_back(std::pair<int,btMultiBodyJointFeedback*>(linkIndex,_jointfb));
    }
    return true;
    
}

void BulletURDFImporter::ParseBaseCollisionMesh()
{
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(p_multibody,-1);
    btCollisionShape* shape = new btBoxShape(btVector3(0.01, 0.01, 0.01));
    col->setCollisionShape(shape);
    
    btTransform tr;
    tr.setOrigin(_base_pos);
    tr.setRotation(_base_rot);
    col->setWorldTransform(tr);

    bool isDynamic = (p_multibody->getBaseMass() == 0 && fixedBase) ? false : true;
    int collisionfiltergroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
    int collisionfiltermask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
    m_world->addCollisionObject(col,collisionfiltergroup, collisionfiltermask); // TO-DO: Need to evaluate other two arguments.
    // m_world->addCollisionObject(col);

    m_multibody->_colShapes.push_back(shape);
    m_multibody->_colliders.push_back(col);
}

void BulletURDFImporter::ParseCollisionMeshes(btMultiBodyDynamicsWorld* m_dynamicsWorld)
{
    // setup world pointer;
    m_world = m_dynamicsWorld;

    std::vector<urdf::Link*> _tptr, _pptr, _cptr;
    std::vector<int> p_ind, c_ind, t_ind;

    int last_ind = 0, child_ind;
    p_ind.push_back(last_ind);
    last_ind++;
    _pptr.push_back(baseLink);

    // createMultiBodyLinkCollisionShapes(0,-1,)
    // first compute forwardKinematics first.
    p_multibody->forwardKinematics(rot_world_to_local,local_origin_world_frame);
    // parse base collision mesh.
    ParseBaseCollisionMesh();
    if (!createMultiBodyLinkCollisionShapes(0, baseLink))
        std::cout << "No collision meshes found for link name: " << baseLink->name << " .Skipping ..." << std::endl;
        // throw std::runtime_error("Failed to build collision shape for link index: "+ std::to_string(0));

    urdf::Link* _ulink;
    urdf::Joint* _ujoint;
   
    while (!_pptr.empty())
    {
        // std::cout << _pptr.size() << std::endl;
        for (int i = 0; i < _pptr.size(); i++)
        {
            for (int j = 0; j < _pptr[i]->child_links.size(); j++)
            {
                _ulink = _pptr[i]->child_links[j].get();
                _cptr.push_back(_ulink);

                c_ind.push_back(last_ind);
                child_ind = last_ind;
                //
                // add code to create a rigidbody in multibody tree here.
                // createMultiBodyCompFromURDFLink(child_ind,p_ind[i],_ulink);
                if (!createMultiBodyLinkCollisionShapes(child_ind, _ulink))
                    std::cout << "No collision meshes found for link name: " << _ulink->name << " .Skipping ..." << std::endl;
                    // throw std::runtime_error("Failed to build collision shape for link index: " + std::to_string(child_ind));
                //
                std::cout << "( " << p_ind[i] <<" " << last_ind << " )" << "\n";
                last_ind++;

            }
        }
        // swap the vectors.
        _tptr = _cptr;
        _cptr = _pptr;
        _pptr = _tptr;

        // swap ind vectors.
        t_ind = c_ind;
        c_ind = p_ind;
        p_ind = t_ind;

        // clear the vector containing children after making it pptr.
        _cptr.clear();
        c_ind.clear();
         
    }
    std::cout << "Parsed collision meshes\n";
    
}
/**
 * The function can resolve path if the folder containing meshes is present in the directory same as
 * the directory containing urdf file. If the mesh filepath contains package directive, the program can
 * resolve the path if the path exists otherwise it will throw up runtime error.
*/
std::string BulletURDFImporter::resolvePath(std::string& _path)
{
    std::filesystem::path path(_path), res;
    if (path.begin()->generic_string() == "package:")
    {
        if (_find_package_path(path,res))
        {
            path = res;
        }
        else
        {
            throw std::runtime_error("Cannot resolve path : " + path.generic_string()); 
        }
        
    }
    else
    {
        if (!std::filesystem::exists(path))
        {
            std::filesystem::path dir_path = URDF_PATH.parent_path();
            try
            {
                path = dir_path.append(path.string());
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                throw std::runtime_error("Cannot resolve path : " + path.generic_string()); 
            }
            
        }
    }
    
    return path.generic_string();
}

bool BulletURDFImporter::_find_package_path(std::filesystem::path& _path, std::filesystem::path& _res)
{
    int count = 20;
    bool _found = false;
    std::string package_string = "package:";
    std::string _resource_path = _path.generic_string();
    std::string _str = _resource_path.replace(_resource_path.find(package_string),package_string.length(),"");
    std::filesystem::path current_path = URDF_PATH;
    std::filesystem::path validpath;

    if (!PACKAGE_PATH.empty())
    {
        _res = std::filesystem::path(PACKAGE_PATH + _str);
        if (std::filesystem::exists(_res))
            return true;
        else
            return false;
    }
    while (count >= 0)
    {
        std::string _str2 = current_path.generic_string() + _str;
        validpath = std::filesystem::path(_str2);
        if (std::filesystem::exists(validpath))
        {
            _res = validpath;
            PACKAGE_PATH = current_path.generic_string();
            _found = true;
            break;
        } 
        current_path = current_path.parent_path();
        count--;
    }
    return _found;
}

bool BulletURDFImporter::_find_collision_mesh_world_transform(btMultiBodyLinkCollider* _col, int linkIndex, btVector3& _colPosThisLinkFrame,
    btQuaternion& _colRotThisLinkFrame, btTransform& tr)
{
    try
    {
        btMultibodyLink _link = p_multibody->getLink(linkIndex);
        btVector3 _dThisLinkComToCol_local = _colPosThisLinkFrame - _link.m_dVector; // distance from col mesh origing to com of this link.
        btVector3 _dThisLinkComtoCol_world =  quatRotate(rot_world_to_local[linkIndex + 1], _dThisLinkComToCol_local );
        btVector3 vec = local_origin_world_frame[linkIndex + 1] + _dThisLinkComtoCol_world;
        printVector(_dThisLinkComToCol_local,"col pos");

        // printVector(local_origin_world_frame[linkIndex + 1],"com world pos");
        // fill up transforms.

        tr.setIdentity();
        tr.setOrigin(local_origin_world_frame[linkIndex + 1] + _dThisLinkComtoCol_world);
        tr.setRotation(rot_world_to_local[linkIndex + 1]);

        // attach distance from com to col origin in local frame to a userpointer.
        btTransform* _ptr = new btTransform();
        _ptr->setOrigin(_dThisLinkComToCol_local);
        _ptr->setRotation(_colRotThisLinkFrame);
        
        // if (_)
        // {
        //     /* code */
        // }
        

        if (_col != nullptr)
        {
            std::cout << "setting user pointer. linkIndex: " << linkIndex << std::endl;
            _col->setUserPointer((void*)_ptr);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << "Error while getting collision mesh tranform of Link Index: " << linkIndex << '\n';
        return false;
        // throw std::runtime_error("Error while computing collision mesh transform. Link Index: " + std::to_string(linkIndex));
    } 
    return true;
}

bool BulletURDFImporter::_find_collision_mesh_local_transform(btMultiBodyLinkCollider* _col, int linkIndex, btVector3& _colPosThisLinkFrame,
    btQuaternion& _colRotThisLinkFrame, btTransform& tr)
{
    try
    {
        btMultibodyLink _link = p_multibody->getLink(linkIndex);
        btVector3 _dThisLinkComToCol_local = _colPosThisLinkFrame - _link.m_dVector; // distance from col mesh origing to com of this link.
        // btVector3 _dThisLinkComtoCol_world =  quatRotate(rot_world_to_local[linkIndex + 0].inverse(), _dThisLinkComToCol_local );
        // btVector3 vec = _dThisLinkComtoCol_world;
        // printVector(vec,"col local pos");
        // fill up transforms.
        tr.setIdentity();
        tr.setOrigin(_dThisLinkComToCol_local);
        tr.setRotation(_colRotThisLinkFrame); // need to check the rotation order.
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << "\nError while getting collision mesh tranform of Link Index: " << linkIndex << '\n';
        return false;
        // throw std::runtime_error("Error while computing collision mesh transform. Link Index: " + std::to_string(linkIndex));
    } 
    return true;
}

mMultiBody* BulletURDFImporter::getMultiBodyStruct(btMultiBodyDynamicsWorld* _m_dynamicsWorld)
{
    m_world = _m_dynamicsWorld;
    ParseCollisionMeshes(m_world);
    return m_multibody;
    
}

void BulletURDFImporter::convertStringToInertiaVector(btMatrix3x3& imat, btVector3& iv)
{
    // imat.diagonalize();
    // TO-DO: Need to implement this function to compute eigen value and eigen vector 
    //        using eigen or bullet linear math.
}