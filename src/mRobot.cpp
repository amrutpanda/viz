
#include "mRobot.h"

namespace mviz
{
    // mRobot::mRobot(std::string robotname, std::string urdf_file, Ogre::SceneManager* _scnMgr, Ogre::SceneNode* root_node)
    mRobot::mRobot(std::string robotname, std::string urdf_file,Ogre::SceneManager* _scnMgr ,Ogre::SceneNode* root_node, 
        Eigen::Vector3d _bpos, Eigen::Quaterniond _brot, bool show_collision)
    {
        robot_name = robotname;
        _urdf_file = urdf_file;
        rootNode = root_node;
        scnMgr = _scnMgr;
        urdf::ModelInterfaceSharedPtr _urdf = urdf::parseURDFFile(_urdf_file);
        std::filesystem::path path = std::filesystem::path(_urdf_file);
        URDF_PATH = path;
        std::cout << "urdf root path : " << URDF_PATH << std::endl;
        _collision_flag = show_collision;
        // _collision_flag = true;
        // initialize transformations and co-ordinates.
        // base_pos.setZero();
        // base_rot.setIdentity();

        base_pos = _bpos;
        base_rot = _brot;

        urdf_root_link = _urdf->getRoot();
        _robot_name = _urdf->getName();

        urdf::Link* urdf_root_link_ptr = const_cast<urdf::Link*>(urdf_root_link.get());
        

        std::cout << "Attempting to convert the urdf object to tree of mRobotLinks." << std::endl;

        createOgreNodesFromLinkPtr(urdf_root_link_ptr,rootNode);

        std::cout << "Total joints: " << joint_name.size() << std::endl;
        std::cout << "Total joints num in urdf: " << _urdf->joints_.size() << std::endl;

        // setRobotAxisVisible(false);
        
    }

    std::string mRobot::getName()
    {
        std::string _name;
        _name = robot_name;
        return _name;
    }
   

    void mRobot::convertVisualLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
    {
        std::cout << "Visual array size: " << ulink->visual_array.size() << std::endl;
        for (int i = 0; i < ulink->visual_array.size(); ++i)
        {
            urdf::VisualSharedPtr vptr = ulink->visual_array[i];
            // if not visual component exists.
            if (vptr.get() != nullptr)
            {
                // std::cout <<   << std::endl;
                bool rqd_mat_loading = true;
                std::string mesh_name;
                double x,y,z,qw,qx,qy,qz;
                x = vptr->origin.position.x;
                y = vptr->origin.position.y;
                z = vptr->origin.position.z;
                vptr->origin.rotation.getQuaternion(qx,qy,qz,qw);
                unsigned int type = getUrdfGeometryType(vptr->geometry);

                switch (type)
                {
                case urdf::Geometry::BOX :
                    {
                        double l,b,h;
                        mesh_name = "mCube.mesh";

                        urdf::Box* box_ptr = dynamic_cast<urdf::Box*>(vptr->geometry.get());
                        l = box_ptr->dim.x;
                        b = box_ptr->dim.y;
                        h = box_ptr->dim.z;

                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(l,b,h));
                        break;
                    } 
                case urdf::Geometry::CYLINDER :
                    {
                        double r,h;
                        mesh_name = "mCylinder.mesh";
                        
                        urdf::Cylinder* c_ptr = dynamic_cast<urdf::Cylinder*>(vptr->geometry.get());

                        r = c_ptr->radius;
                        h = c_ptr->length;
                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(r,r,h/10));
                        
                        break;
                    }
                case urdf::Geometry::SPHERE :
                    {
                        double r;
                        mesh_name = "mSphere.mesh";
                        urdf::Sphere* s_ptr = dynamic_cast<urdf::Sphere*>(vptr->geometry.get());

                        r = s_ptr->radius;

                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(r,r,r));

                        break;
                    }
                
                case urdf::Geometry::MESH :
                    {
                        std::string mesh_file_name;
                        urdf::Mesh* m_ptr = dynamic_cast<urdf::Mesh*>(vptr->geometry.get());
                        
                        std::filesystem::path dir_path(_urdf_file), filepath(m_ptr->filename);
                        // mesh_file_name = dir_path.parent_path().append(filepath.string());
                        mesh_file_name = resolvePath(m_ptr->filename);
                        createMeshFromFile(mesh_file_name,mesh_name);

                        // check whether the file extension is stl and set material loading flag true;
                        if (filepath.extension() == ".stl" )
                            rqd_mat_loading = true;
                        else
                            rqd_mat_loading = false;


                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(m_ptr->scale.x, m_ptr->scale.y, m_ptr->scale.z));

                        if (filepath.extension() == ".dae" || filepath.extension() == ".DAE")
                        {
                            // A patch to handle orientatin mismatch issue with Collada files.
                            // Change the angle and axes if you see mesh pose discrepancy.
                            std::cout << "Found a Collada file." << std::endl;
                            // rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_X));
                            rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(mfAngle),mfAxis));
                        }
                        break;
                    }
                
                default:
                    {
                        rqd_mat_loading = false;
                        std::cout << "No Visual Info found\n";
                        break;
                    }
                }
           
                // set material 
                if (rqd_mat_loading && (vptr->material.get() != nullptr))
                {
                    std::cout << "Preparing to load material..." << std::endl;
                    Ogre::MaterialPtr pmat;
                    Ogre::ResourceManager::ResourceCreateOrRetrieveResult res;
                    if (vptr->material->name.empty())
                        vptr->material->name = "Default";
                    pmat = Ogre::MaterialManager::getSingleton().getByName(vptr->material->name);
                    if (pmat.get() == nullptr)
                        pmat = Ogre::MaterialManager::getSingleton().create(vptr->material->name,"UserData");
                    // TO-DO: Need to fix a bug here.
                    Ogre::ColourValue _color = Ogre::ColourValue(vptr->material->color.r,
                                                                vptr->material->color.g,
                                                                vptr->material->color.b);
                    pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
                    pmat->getTechnique(0)->getPass(0)->setDiffuse(vptr->material->color.r,
                                                                   vptr->material->color.g,
                                                                vptr->material->color.b,0.9);
                    // pmat->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SceneBlendType::SBT_TRANSPARENT_COLOUR);
                    // pmat->getTechnique(0)->getPass(0)->setSceneBlendingOperation(Ogre::SceneBlendOperation::SBO_ADD);
                    pmat->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBF_SOURCE_ALPHA,Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
                    pmat->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
                   
        
                    if (!vptr->material->texture_filename.empty())
                    {
                        pmat->getTechnique(0)->getPass(0)->createTextureUnitState(vptr->material->texture_filename);
                    }
                    // retrieve the attached entity from the child mesh node;
                    Ogre::Entity* ent = static_cast<Ogre::Entity*>(rlink->getChildMeshNode(mesh_name)->getAttachedObject(0));
                    ent->setMaterial(pmat);
                }
                else if (rqd_mat_loading)
                {
                    // std::cout << "Loading default material." << std::endl;
                    Ogre::Entity* ent = static_cast<Ogre::Entity*>(rlink->getChildMeshNode(mesh_name)->getAttachedObject(0));
                    ent->setMaterialName("Collision");
                }
                else
                    std::cout << "No material required \n";
                // store the mesh names.

                rlink->meshes.push_back(mesh_name);
            }
        }   
        
    }

    /**
     * Converts a collision information of urdf link to mRobotLink for visualization.
    */

    void mRobot::convertCollisionLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
    {
        std::cout << "Collision array size: " << ulink->collision_array.size() << std::endl;
        for (int i = 0; i < ulink->collision_array.size(); ++i)
        {
            urdf::CollisionSharedPtr cptr = ulink->collision_array[i];
            // if not Collision component exists.
            if (cptr.get() != nullptr)
            {
                // std::cout <<   << std::endl;
                bool rqd_mat_loading = true;
                std::string mesh_name;
                double x,y,z,qw,qx,qy,qz;
                x = cptr->origin.position.x;
                y = cptr->origin.position.y;
                z = cptr->origin.position.z;
                cptr->origin.rotation.getQuaternion(qx,qy,qz,qw);
                unsigned int type = getUrdfGeometryType(cptr->geometry);

                switch (type)
                {
                case urdf::Geometry::BOX :
                    {
                        double l,b,h;
                        mesh_name = "mCube.mesh";

                        urdf::Box* box_ptr = dynamic_cast<urdf::Box*>(cptr->geometry.get());
                        l = box_ptr->dim.x;
                        b = box_ptr->dim.y;
                        h = box_ptr->dim.z;

                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(l,b,h));
                        break;
                    } 
                case urdf::Geometry::CYLINDER :
                    {
                        double r,h;
                        mesh_name = "mCylinder.mesh";
                        
                        urdf::Cylinder* c_ptr = dynamic_cast<urdf::Cylinder*>(cptr->geometry.get());

                        r = c_ptr->radius;
                        h = c_ptr->length;
                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(r,r,h/10));
                        
                        break;
                    }
                case urdf::Geometry::SPHERE :
                    {
                        double r;
                        mesh_name = "mSphere.mesh";
                        urdf::Sphere* s_ptr = dynamic_cast<urdf::Sphere*>(cptr->geometry.get());

                        r = s_ptr->radius;

                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(r,r,r));

                        break;
                    }
                
                case urdf::Geometry::MESH :
                    {
                        std::string mesh_file_name;
                        urdf::Mesh* m_ptr = dynamic_cast<urdf::Mesh*>(cptr->geometry.get());
                        
                        std::filesystem::path dir_path(_urdf_file), filepath(m_ptr->filename);
                        // mesh_file_name = dir_path.parent_path().append(filepath.string());
                        mesh_file_name = resolvePath(m_ptr->filename);
                        createMeshFromFile(mesh_file_name,mesh_name);

                        // check whether the file extension is stl and set material loading flag true;
                        if (filepath.extension() == ".stl" )
                            rqd_mat_loading = true;
                        else
                            rqd_mat_loading = false;


                        rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                                Ogre::Vector3(m_ptr->scale.x, m_ptr->scale.y, m_ptr->scale.z));

                        if (filepath.extension() == ".dae" || filepath.extension() == ".DAE")
                        {
                            // A patch to handle orientatin mismatch issue with Collada files.
                            // Change the angle and axes if you see mesh pose discrepancy.
                            std::cout << "Found a Collada file." << std::endl;
                            // rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_X));
                            rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(mfAngle),mfAxis));
                        }
                        break;
                    }
                
                default:
                    {
                        rqd_mat_loading = false;
                        std::cout << "No Collision Info found\n";
                        break;
                    }
                }
                // set material 

                std::cout << "Preparing to load material..." << std::endl;
                Ogre::MaterialPtr pmat;
                Ogre::ResourceManager::ResourceCreateOrRetrieveResult res;
                pmat = Ogre::MaterialManager::getSingleton().getByName("Collision");
                // retrieve the attached entity from the child mesh node;
                Ogre::Entity* ent = static_cast<Ogre::Entity*>(rlink->getChildMeshNode(mesh_name)->getAttachedObject(0));
                ent->setMaterial(pmat);

                rlink->meshes.push_back(mesh_name);
            }
        }   
        
    }



    void mRobot::createOgreNodesFromLinkPtr(urdf::Link* ulink,Ogre::SceneNode* ogNode)
    {
        if (ulink->child_links.size() == 0)
        {
            std::cout << "++++++++++++++++++++++++++++++++++" << std::endl;
            Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
            link_names.push_back(ulink->name);
            ogreNodes.push_back(childNode);
            // create a mRobotLink object.
            mRobotLink* rlink = new mRobotLink();
            object_ptrs.push_back(rlink);
            rlink->objectName = ulink->name;
            rlink->setSceneNode(childNode);

            childNode = rlink->getSceneNode();
            
            // axis test.
            // axis* ax = new axis(rlink);
            rlink->setAxis();
            // convertVisualLinkPtrTomRobotLink(ulink,rlink);
            // convertCollisionLinkPtrTomRobotLink(ulink,rlink);

            if (!_collision_flag)
                convertVisualLinkPtrTomRobotLink(ulink,rlink);
            else
                convertCollisionLinkPtrTomRobotLink(ulink,rlink);
        
            if (ulink->parent_joint != nullptr)
            {
                ParseJoint(rlink,ulink->parent_joint);
            }
            std::cout << "Parsed link: " << ulink->name << std::endl;
        }
        else 
        {
            std::cout << "=====================================" << std::endl;

            Ogre::SceneNode* childNode = ogNode->createChildSceneNode();

            link_names.push_back(ulink->name);
            
            ogreNodes.push_back(childNode);
            // create a mRobotLink object.
            mRobotLink* rlink = new mRobotLink();
            object_ptrs.push_back(rlink);
            rlink->objectName = ulink->name;
            rlink->setSceneNode(childNode);

            childNode = rlink->getSceneNode();              // this statement changed everything.

                // axis test.
            // axis* ax = new axis(rlink);
            rlink->setAxis();
            rlink->setAxisVisible(true);
            
            if (!_collision_flag)
                convertVisualLinkPtrTomRobotLink(ulink,rlink);
            else
                convertCollisionLinkPtrTomRobotLink(ulink,rlink);

            if (ulink->parent_joint != nullptr)
            {
                ParseJoint(rlink,ulink->parent_joint);
            }
            else 
            {
                ParseBaseLink(rlink);
            }

            std::cout << "Parsed link: " << ulink->name << std::endl;
            for (size_t i = 0; i < ulink->child_links.size(); i++)
            {
                // recursive call to this function.
                // if (link_names.size() >= 4)
                // {
                //     std::cout << "Stopping now" << std::endl;
                //     break;
                // }
                createOgreNodesFromLinkPtr(ulink->child_links[i].get(),childNode);
            }
            
        }
    }

    void mRobot::ParseJoint(mRobotLink* _rlink, urdf::JointConstSharedPtr _jptr)
    {
        std::cout << _jptr->name << std::endl;
        // get joint type and joint axis.
        _rlink->type = _jptr->type;
        // _rlink->axis = Eigen::Vector3d(_jptr->axis.x, _jptr->axis.y, _jptr->axis.z);
        _rlink->_axis = Ogre::Vector3(_jptr->axis.x, _jptr->axis.y, _jptr->axis.z);
        // get the position values from JointSharedPtr
        double x = _jptr->parent_to_joint_origin_transform.position.x;
        double y = _jptr->parent_to_joint_origin_transform.position.y;
        double z = _jptr->parent_to_joint_origin_transform.position.z;
        // set the position values to mRobotLink object.
        double qw,qx,qy,qz;
        _jptr->parent_to_joint_origin_transform.rotation.getQuaternion(qx,qy,qz,qw);

        _rlink->setPosition(Ogre::Vector3(x,y,z));
        _rlink->setRotation(qw,qx,qy,qz);
        
        // push data to lists excluding the FIXED joints as no need to update them.
        // if (_jptr->type != urdf::Joint::FIXED || _jptr->type != urdf::Joint::FLOATING || _jptr->type != urdf::Joint::PLANAR)
        if (_jptr->type == urdf::Joint::REVOLUTE || _jptr->type == urdf::Joint::CONTINUOUS || _jptr->type == urdf::Joint::PRISMATIC)
        {
            std::cout << "JOINT NAME: " << _jptr->name << std::endl;
            joint_name.push_back(_jptr->name);
            joint_type.push_back(_jptr->type);
            // save the pointer to joint value of the link;
            joint_value_holder.push_back(&_rlink->joint_variable);        
        }
        else
        {
            std::cout << "found a FIXED Joint\n";
        }
        
    }

    void mRobot::ParseVisualInfo(mRobotLink* _rlink, urdf::VisualSharedPtr _vptr)
    {
        double x,y,z, qw,qx,qy,qz;
        x = _vptr->origin.position.x;
        y = _vptr->origin.position.y;
        z = _vptr->origin.position.z;

        qw = _vptr->origin.rotation.w;
        qx = _vptr->origin.rotation.x;
        qy = _vptr->origin.rotation.y;
        qz = _vptr->origin.rotation.z;

        Eigen::Vector3d t(x,y,z);
        Eigen::Quaterniond Q(qw,qx,qy,qz);

        // creating mesh from the given file name;
        std::string entity_name;
        createMeshFromFile(_rlink->mesh_file_name,entity_name);
        _rlink->attachChildMesh(scnMgr, entity_name,Ogre::Vector3(x,y,z), Ogre::Quaternion(qw,qx,qy,qz));
        

    }

    void mRobot::ParseBaseLink(mRobotLink* _rlink)
    {
        // set _rlink type to be unknown.
        _rlink->type = urdf::Joint::UNKNOWN;
        // set _rlink position and rotation.
        _rlink->setPosition(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
        _rlink->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());

    }

    void mRobot::updateRobot(Eigen::VectorXd& robot_pos )
    {
        assert(robot_pos.size() == joint_name.size());

        for (int i = 0; i < joint_name.size(); i++)
        {
            *joint_value_holder[i] = robot_pos[i];
        }

        mRobotLink* mRobotLinkPtr;
        int type;
        for (int i = 0; i < object_ptrs.size(); i++)
        {
            mRobotLinkPtr = object_ptrs[i];
            type = mRobotLinkPtr->type;
        
            if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS)
            {   
                mRobotLinkPtr->setRotationLocal(Ogre::Quaternion(Ogre::Radian(mRobotLinkPtr->joint_variable),mRobotLinkPtr->_axis));   
            }
            else if (type == urdf::Joint::PRISMATIC)
            {
                mRobotLinkPtr->setPositionLocal(mRobotLinkPtr->_axis * Ogre::Vector3(mRobotLinkPtr->joint_variable));
            }

            else if((type == urdf::Joint::FIXED) && (i == 0)) // Not used at this moment. Will be changed in future versions.
            {
                mRobotLinkPtr->setPosition(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
                mRobotLinkPtr->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());
            }
            else if((type == urdf::Joint::UNKNOWN) && (i == 0))
            {
                // std::cout << "Seems to be the base link\n" << std::endl;
                mRobotLinkPtr->setPositionLocal(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
                mRobotLinkPtr->setRotationLocal(Ogre::Quaternion(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z()));
            }
        }     
    }

    unsigned int mRobot::getUrdfGeometryType(urdf::GeometrySharedPtr _gptr)
    {
        if (dynamic_cast<urdf::Box*>(_gptr.get()))
            return urdf::Geometry::BOX;
        if (dynamic_cast<urdf::Cylinder*>(_gptr.get()))
            return urdf::Geometry::CYLINDER;
        if (dynamic_cast<urdf::Sphere*>(_gptr.get()))
            return urdf::Geometry::SPHERE;
        if (dynamic_cast<urdf::Mesh*>(_gptr.get()))
            return urdf::Geometry::MESH;
        return 100;
    }
    

    void mRobot::setBasePose(Eigen::Vector3d _pose)
    {
        base_pos = _pose;
    }

    void mRobot::setBaseRotation(Eigen::Quaterniond _qRotation)
    {
        base_rot = _qRotation;
    }

    void mRobot::setRobotAxisVisible(bool _flag)
    {
        mRobotLink* _rlink;
        for (int i = 0; i < link_names.size(); i++)
        {
            _rlink = object_ptrs[i];
            _rlink->setAxisVisible(_flag);
        }
        
    }
    int mRobot::getRobotNumJoints()
    {
        return joint_name.size();
    }

    mRobotLink* mRobot::getRobotLinkFromFrameName(std::string& _fName)
    {
        for (int i = 0; i < link_names.size(); i++)
        {
            if (link_names[i] == _fName)
            {
                return object_ptrs[i];
            }
        }
        return nullptr;
        
    }

    void mRobot::flipDAEMeshes(double& angle, int axis)
    {
        mfAngle = angle;
        if (axis == X)
        {
            mfAxis = Ogre::Vector3::UNIT_X;
            std::cout << "Fliping the X axis" << std::endl;
        }
        else if (axis == Y)
        {
            mfAxis = Ogre::Vector3::UNIT_Y;
            std::cout << "Fliping the Y axis" << std::endl;
        }
        else if (axis == Z)
        {
            mfAxis = Ogre::Vector3::UNIT_Z;
            std::cout << "Fliping the Z axis" << std::endl;
        }
        else
        {
            throw std::runtime_error("Invalid axis assignment");
        }
        std::cout << "Angle: " << angle << " mfAngle: " << mfAngle << std::endl; 
        for (int i = 0; i < object_ptrs.size(); i++)
        {
            mRobotLink* rlink = object_ptrs[i];
           for (int j = 0; j < rlink->meshes.size(); j++)
           {
                // std::filesystem::path Path(rlink->meshes[i]);
                Ogre::SceneNode* _tNode = rlink->getChildMeshNode(rlink->meshes[j]);
                _tNode->rotate(Ogre::Quaternion(Ogre::Degree(angle),mfAxis));
           }
           
        }

    }

    mRobot::~mRobot()
    {
        std::cout << "deleting " << getName() << std::endl;
        for (int i = object_ptrs.size() -1 ; i >= 0; i--)
        {
            delete object_ptrs[i];
        }
        // rootNode->getParentSceneNode()->removeAndDestroyChild(rootNode);
        std::cout << "Robot Object destoryed" << std::endl;
    }

    /**
     * The function can resolve path if the folder containing meshes is present in the directory same as
     * the directory containing urdf file. If the mesh filepath contains package directive, the program can
     * resolve the path if the path exists otherwise it will throw up runtime error.
    */
    std::string mRobot::resolvePath(std::string& _path)
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

    bool mRobot::_find_package_path(std::filesystem::path& _path, std::filesystem::path& _res)
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

} // namespace mviz
