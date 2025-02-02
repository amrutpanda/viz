#include "mRobot.h"


namespace mviz
{
    mRobot::mRobot(std::string robotname, std::string urdf_file, Ogre::SceneManager* _scnMgr, Ogre::SceneNode* root_node)
    {
        robot_name = robotname;
        _urdf_file = urdf_file;
        rootNode = root_node;
        scnMgr = _scnMgr;
        urdf::ModelInterfaceSharedPtr _urdf = urdf::parseURDFFile(_urdf_file);

        // initialize transformations and co-ordinates.
        T_var.setIdentity();
        base_pos.setZero();
        base_rot.setIdentity();

        urdf_root_link = _urdf->getRoot();
        _robot_name = _urdf->getName();

        urdf::Link* urdf_root_link_ptr = const_cast<urdf::Link*>(urdf_root_link.get());

        std::cout << "Attempting to convert the urdf object to tree of mRobotLinks." << std::endl;

        createOgreNodesFromLinkPtr(urdf_root_link_ptr,rootNode);
        
    }

    std::string mRobot::getName()
    {
        std::string _name;
        _name = robot_name;
        return _name;
    }
   


    void mRobot::convertLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
    {
        urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
        if (m == NULL)
        {
            std::cout << "At this moment only Mesh objects are supported." << std::endl;
            throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

        }
        // rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();

        std::filesystem::path dir_path(_urdf_file);

        // rlink->mesh_file_name = dir_path.parent_path(). + std::filesystem::path(m->filename);
        rlink->mesh_file_name = dir_path.parent_path().append(std::filesystem::path(m->filename).string());
        
        std::cout << ulink->name << " :: " << rlink->mesh_file_name << std::endl;
        rlink->scale << m->scale.x , m->scale.y, m->scale.z;
    }

    // void mRobot::convertLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
    // {
    //     urdf::VisualSharedPtr vptr = ulink->visual;
    //     // if not visual component exists.
    //     if (vptr.get() == nullptr)
    //     {
    //         rlink->setPosition(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
    //         rlink->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());
    //         rlink->type = urdf::Joint::FIXED;
    //     }
    //     else
    //     {
    //         double x,y,z,qw,qx,qy,qz;
    //         x = vptr->origin.position.x;
    //         y = vptr->origin.position.y;
    //         z = vptr->origin.position.z;
    //         vptr->origin.rotation.getQuaternion(qx,qy,qz,qw);
    //         unsigned int type = ulink->visual->geometry->type;
    //         switch (type)
    //         {
    //         case urdf::Geometry::BOX :
    //             {
    //                 double l,b,h;
    //                 std::string mesh_name = "mCube.mesh";
                     
    //                 urdf::Box* box_ptr = dynamic_cast<urdf::Box*>(vptr->geometry.get());
    //                 l = box_ptr->dim.x;
    //                 b = box_ptr->dim.y;
    //                 h = box_ptr->dim.z;

    //                 rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
    //                                         Ogre::Vector3(l,b,h));
    //                 break;
    //             } 
    //         case urdf::Geometry::CYLINDER :
    //             {
    //                 double r,h;
    //                 std::string mesh_name = "mCylinder.mesh";
    //                 urdf::Cylinder* c_ptr = dynamic_cast<urdf::Cylinder*>(vptr->geometry.get());

    //                 r = c_ptr->radius;
    //                 h = c_ptr->length;

    //                 rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
    //                                         Ogre::Vector3(r,r,h/10));
    //                 break;
    //             }
    //         case urdf::Geometry::SPHERE :
    //             {
    //                 double r;
    //                 std::string mesh_name = "mSphere.mesh";
    //                 urdf::Sphere* s_ptr = dynamic_cast<urdf::Sphere*>(vptr->geometry.get());

    //                 r = s_ptr->radius;

    //                 rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
    //                                         Ogre::Vector3(r,r,r));
    //                 break;
    //             }
            
    //         case urdf::Geometry::MESH :
    //             {
    //                 std::string mesh_file_name, mesh_name;
    //                 urdf::Mesh* m_ptr = dynamic_cast<urdf::Mesh*>(vptr->geometry.get());
                    
    //                 std::filesystem::path dir_path(_urdf_file);
    //                 mesh_file_name = dir_path.parent_path().append(std::filesystem::path(m_ptr->filename).string());
    //                 creatMeshFromFile(mesh_file_name,mesh_name);

    //                 rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
    //                                         Ogre::Vector3(m_ptr->scale.x,m_ptr->scale.y, m_ptr->scale.z));
    //             }
            
    //         default:
    //             break;
    //         }
    //     }
        
    // }

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
            
            // axis test.
            // axis* ax = new axis(rlink);
            rlink->setAxis();

            if (ulink->visual.get() != nullptr)
            {
                // convert the urdf link object to mRobotLink object.
                convertLinkPtrTomRobotLink(ulink,rlink);
                ParseVisualInfo(rlink,ulink->visual);
                // creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);
                // rlink->createEntity(scnMgr); // an axis object will be attached here at later stage.

                // rlink->setMaterialColor(Ogre::ColourValue(1.0, 0.4, 0.1, 1.0));

            }
            else
            {
                rlink->T_visual.setIdentity();
            }

            // convert the urdf link object to mRobotLink object.
            // convertLinkPtrTomRobotLink(ulink,rlink);

            // ParseVisualInfo(rlink,ulink->visual);
            
            // rlink->setSceneNode(childNode);

            if (ulink->parent_joint != nullptr)
            {
                ParseJoint(rlink,ulink->parent_joint);
            }
            // rlink->objectName = ulink->name;

            std::cout << "Parsed link: " << ulink->name << std::endl;
            // creatMeshFromFile(rlink->mesh_file_name, rlink->entity_name);
            // rlink->createEntity(scnMgr);
        }
        else 
        {
            std::cout << "=====================================" << std::endl;
            for (size_t i = 0; i < ulink->child_links.size(); i++)
            {
                Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
                link_names.push_back(ulink->name);
                
                ogreNodes.push_back(childNode);
                // create a mRobotLink object.
                mRobotLink* rlink = new mRobotLink();
                object_ptrs.push_back(rlink);
                rlink->objectName = ulink->name;
                rlink->setSceneNode(childNode);

                 // axis test.
                // axis* ax = new axis(rlink);
                rlink->setAxis();
                rlink->setAxisVisible(true);

                if (ulink->visual.get() != nullptr)
                {
                    // convert the urdf link object to mRobotLink object.
                    convertLinkPtrTomRobotLink(ulink,rlink);
                    ParseVisualInfo(rlink,ulink->visual);
                    // creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);
                    // rlink->createEntity(scnMgr); // an axis mesh will be attached at the later stage.

                    // rlink->attachChildMesh(scnMgr,rlink->entity_name,Ogre::Vector3())
                    // rlink->setMaterialColor(Ogre::ColourValue(1.0, 1.0/(i+1), 0.1 + 0.01*i, 1.0));

                }
                else
                {
                    rlink->T_visual.setIdentity();
                }
                
                // convertLinkPtrTomRobotLink(ulink,rlink);
                
                // ParseVisualInfo(rlink,ulink->visual);
                
                // rlink->setSceneNode(childNode);
                if (ulink->parent_joint != nullptr)
                {
                    ParseJoint(rlink,ulink->parent_joint);
                }
                else 
                {
                    ParseBaseLink(rlink);
                }
                
                // rlink->objectName = ulink->name;
                // creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);
                // rlink->createEntity(scnMgr);
                // set material. EXPERIMENTAL
                // rlink->setMaterialColor(Ogre::ColourValue(1.0, 0.4, 0.1, 1.0));

                std::cout << "Parsed link: " << ulink->name << std::endl;
                // recursive call to this function.
                createOgreNodesFromLinkPtr(ulink->child_links[i].get(),childNode);
            }
            
        }
    }

    void mRobot::ParseJoint(mRobotLink* _rlink, urdf::JointConstSharedPtr _jptr)
    {
        std::cout << _jptr->name << std::endl;
        // get joint type and joint axis.
        _rlink->type = _jptr->type;
        _rlink->axis = Eigen::Vector3d(_jptr->axis.x, _jptr->axis.y, _jptr->axis.z);
        _rlink->_axis = Ogre::Vector3(_jptr->axis.x, _jptr->axis.y, _jptr->axis.z);
        // get the position values from JointSharedPtr
        double x = _jptr->parent_to_joint_origin_transform.position.x;
        double y = _jptr->parent_to_joint_origin_transform.position.y;
        double z = _jptr->parent_to_joint_origin_transform.position.z;
        // set the position values to mRobotLink object.
        double qw,qx,qy,qz;
        _jptr->parent_to_joint_origin_transform.rotation.getQuaternion(qx,qy,qz,qw);

        // Eigen::Quaterniond Q(qw,qx,qy,qz);
        // Eigen::Vector3d v(x,y,z), t;
        // Eigen::Affine3d T;
        
        // _rlink->T_p_l.translation() = v;
        // _rlink->T_p_l.linear() = Q.matrix();

        // T = T_var.inverse() * _rlink->T_p_l * _rlink->T_visual;
        // T_var = _rlink->T_visual;

        // t = T.translation();
        // Eigen::Quaterniond Qnew(T.rotation());
        // Eigen::Vector4d q = Qnew.coeffs();

        // _rlink->setPosition(t);
        // _rlink->setRotation(q.w(),q.x(),q.y(),q.z());

        _rlink->setPosition(Ogre::Vector3(x,y,z));
        _rlink->setRotation(qw,qx,qy,qz);
        
        // push data to lists excluding the FIXED joints as no need to update them.
        if (_jptr->type != urdf::Joint::FIXED || _jptr->type != urdf::Joint::FLOATING || _jptr->type != urdf::Joint::PLANAR)
        {
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
        creatMeshFromFile(_rlink->mesh_file_name,entity_name);
        _rlink->attachChildMesh(scnMgr, entity_name,Ogre::Vector3(x,y,z), Ogre::Quaternion(qw,qx,qy,qz));
        
        _rlink->T_visual.translation() = t;
        _rlink->T_visual.linear() = Q.matrix();

        // _vptr->material->color.

    }

    void mRobot::ParseBaseLink(mRobotLink* _rlink)
    {
        // std::cout << base_pos << std::endl;
        // std::cout << base_rot.coeffs() << std::endl;

        _rlink->T_p_l.translation() = base_pos;
        _rlink->T_p_l.linear() = base_rot.matrix();

        _rlink->setPosition(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
        _rlink->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());

        // _rlink->setPosition(base_pos);
        // _rlink->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());

        T_var = _rlink->T_visual;
        _rlink->type = urdf::Joint::FIXED;

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
        // Eigen::Vector3d vRot = Eigen::Vector3d::Zero();
        // Eigen::Vector3d vTrans = Eigen::Vector3d::Zero();
        // Eigen::Vector3d t;
        // Eigen::Matrix3d R;

        // Eigen::Affine3d T;

        // set T_var to be Identity;
        // T_var.setIdentity();

        for (int i = 0; i < object_ptrs.size(); i++)
        {
            mRobotLinkPtr = object_ptrs[i];
            type = mRobotLinkPtr->type;

            if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS)
            {
                // mRobotLinkPtr->T_p_l.linear() = Eigen::AngleAxisd(mRobotLinkPtr->joint_variable,mRobotLinkPtr->axis).matrix();
                mRobotLinkPtr->setRotation(Ogre::Quaternion(Ogre::Radian(mRobotLinkPtr->joint_variable),mRobotLinkPtr->_axis));
            }
            else if (type == urdf::Joint::PRISMATIC)
            {
                // vTrans = mRobotLinkPtr->joint_variable * mRobotLinkPtr->axis;
                // mRobotLinkPtr->T_p_l.translation() = vTrans;
                // Ogre::Vector3 s = mRobotLinkPtr->_axis * Ogre::Vector3(mRobotLinkPtr->joint_variable);
                mRobotLinkPtr->setPosition(mRobotLinkPtr->_axis * Ogre::Vector3(mRobotLinkPtr->joint_variable));
            }

            // else if ((type == urdf::Joint::FIXED) && (i != 0))
            // {
            //     // T_var = mRobotLinkPtr->T_visual;
            //     // continue;
            // }
            else if((type == urdf::Joint::FIXED) && (i == 0))
            {
                // T_var = mRobotLinkPtr->T_visual;
                // mRobotLinkPtr->T_p_l.translation() = base_pos;
                // mRobotLinkPtr->T_p_l.linear() = base_rot.matrix();

                mRobotLinkPtr->setPosition(Ogre::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
                mRobotLinkPtr->setRotation(base_rot.w(), base_rot.x(), base_rot.y(), base_rot.z());
            }

            // T = T_var.inverse() * mRobotLinkPtr->T_p_l * mRobotLinkPtr->T_visual;
            // T_var = mRobotLinkPtr->T_visual;

            // t = T.translation();
            // R = T.linear();

            // mRobotLinkPtr->setPosition(t);

            // Eigen::Quaterniond Qnew(T.rotation());
            // Eigen::Vector4d q = Qnew.coeffs();

            // mRobotLinkPtr->setRotation(q.w(),q.x(), q.y(), q.z());
            // std::cout << mRobotLinkPtr->objectName << std::endl;
        }     
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

} // namespace mviz
