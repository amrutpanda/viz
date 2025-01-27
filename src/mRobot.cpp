#include "mRobot.h"


namespace mviz
{
    mRobot::mRobot(std::string robotname, std::string urdf_file, Ogre::SceneManager* _scnMgr, Ogre::SceneNode* root_node)
    {
        robot_name = robotname;
        _urdf_file = urdf_file;
        rootNode = root_node;
        scnMgr = _scnMgr;
        _urdf = urdf::parseURDFFile(_urdf_file);

        T_var.setIdentity();

        // urdf::LinkConstSharedPtr linkPtr = _urdf->getRoot();
        // urdf_link = linkP

        urdf_root_link = _urdf->getRoot();
        _robot_name = _urdf->getName();


        urdf::Link* urdf_root_link_ptr = const_cast<urdf::Link*>(urdf_root_link.get());

        std::cout << "Attempting to convert the urdf object to tree of mRobotLinks." << std::endl;

        createOgreNodesFromLinkPtr(urdf_root_link_ptr,rootNode);
        
        Eigen::VectorXd robot_pos(7);
        // robot_pos << 0, 10, 0.5, 0.5, 0.7, 0.3, 0.8;
        // robot_pos << 0, 0.1, 0, 0, 0, 0, 0;
        // updateRobot(robot_pos); 
    }

    std::string mRobot::getName()
    {
        std::string _name;
        _name = robot_name;
        return _name;
    }
   
    // void mRobot::convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr ulink, mRobotLink* rlink)
    // {
        
    //     urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
    //     if (m == NULL)
    //     {
    //         std::cout << "At this moment only Mesh objects are supported." << std::endl;
    //         throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

    //     }

    //     rlink->type = Type::MESH; // this is redundant.
    //     // rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();

    //     rlink->mesh_file_name = std::filesystem::path(m->filename).string();

    //     std::cout << ulink->name << " :: " << rlink->mesh_file_name << std::endl;
    //     rlink->scale << m->scale.x , m->scale.y, m->scale.z;
        
    // }


    // void mRobot::convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr ulink, mRobotLink* rlink)
    // {
    //     urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
    //     if (m == NULL)
    //     {
    //         std::cout << "At this moment only Mesh objects are supported." << std::endl;
    //         throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

    //     }
    //     rlink->type = Type::MESH; // this is redundant.
    //     // rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();

    //     rlink->mesh_file_name = std::filesystem::path(m->filename).string();
        
    //     std::cout << ulink->name << " :: " << rlink->mesh_file_name << std::endl;
    //     rlink->scale << m->scale.x , m->scale.y, m->scale.z;
        
    // }

    void mRobot::convertLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
    {
        urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
        if (m == NULL)
        {
            std::cout << "At this moment only Mesh objects are supported." << std::endl;
            throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

        }
        rlink->type = Type::MESH; // this is redundant.
        // rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();

        std::filesystem::path dir_path(_urdf_file);

        // rlink->mesh_file_name = dir_path.parent_path(). + std::filesystem::path(m->filename);
        rlink->mesh_file_name = dir_path.parent_path().append(std::filesystem::path(m->filename).string());
        
        std::cout << ulink->name << " :: " << rlink->mesh_file_name << std::endl;
        rlink->scale << m->scale.x , m->scale.y, m->scale.z;
    }

    // void mRobot::createOgreNodesFromLinkSharedPtr(urdf::LinkSharedPtr ulink, Ogre::SceneNode* ogNode)
    // {
    //     if (ulink->child_links.size() == 0)
    //     {
    //         Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
    //         link_names.push_back(ulink->name);
    //         ogreNodes.push_back(childNode);
    //         // create a mRobotLink object.
    //         mRobotLink* rlink = new mRobotLink();
    //         object_ptrs.push_back(rlink);
    //         // convert the urdf link object to mRobotLink object.

    //         if (ulink->getParent() == nullptr)
    //         {
    //             std::cout << ulink->name << "is the root link." << std::endl;
    //             convertLinkConstSharedPtrTomRobotLink(ulink,rlink);
    //         }
    //         else
    //         {
    //             convertLinkSharedPtrTomRobotLink(ulink,rlink);
    //         }
            
    //         rlink->setSceneNode(childNode);
    //         std::cout << "Parsing link: " << ulink->name << std::endl;
    //         creatMeshFromFile(rlink->mesh_file_name, rlink->entity_name);
    //     }
    //     else 
    //     {
    //         std::cout << "=====================================" << std::endl;
    //         for (size_t i = 0; i < ulink->child_links.size(); i++)
    //         {
    //             Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
    //             link_names.push_back(ulink->name);
    //             ogreNodes.push_back(childNode);
    //             // create a mRobotLink object.
    //             mRobotLink* rlink = new mRobotLink();
    //             object_ptrs.push_back(rlink);
    //             // convert the urdf link object to mRobotLink object.

    //             if (ulink->getParent() == nullptr)
    //             {
    //                 std::cout << ulink->name << "is the root link." << std::endl;
    //                 convertLinkConstSharedPtrTomRobotLink(ulink,rlink);
    //             }
    //             else
    //             {
    //                 convertLinkSharedPtrTomRobotLink(ulink,rlink);
    //             }

    //             // convertLinkSharedPtrTomRobotLink(ulink,rlink);

    //             creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);

    //             std::cout << "Parsing link: " << ulink->name << std::endl;
    //             // recursive call to this function.
    //             createOgreNodesFromLinkSharedPtr(ulink->child_links[i],childNode);
    //         }

    //         std::cout << "---------------------------------------" << std::endl;
            
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
            // convert the urdf link object to mRobotLink object.

            convertLinkPtrTomRobotLink(ulink,rlink);

            ParseVisualInfo(rlink,ulink->visual);
            
            rlink->setSceneNode(childNode);

            if (ulink->parent_joint != nullptr)
            {
                ParseJoint(rlink,ulink->parent_joint);
            }
            rlink->objectName = ulink->name;

            std::cout << "Parsing link: " << ulink->name << std::endl;
            creatMeshFromFile(rlink->mesh_file_name, rlink->entity_name);
            rlink->createEntity(scnMgr);
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
                // convert the urdf link object to mRobotLink object.
                convertLinkPtrTomRobotLink(ulink,rlink);
                
                ParseVisualInfo(rlink,ulink->visual);
                
                rlink->setSceneNode(childNode);
                if (ulink->parent_joint != nullptr)
                {
                    ParseJoint(rlink,ulink->parent_joint);
                }
                else 
                {
                    Eigen::Vector3d v;
                    v = v.Zero();
                    // v(0) = 10;
                    Eigen::Quaterniond q(1,0,0,0);
                    rlink->T_p_l.translation() = v;
                    rlink->T_p_l.linear() = q.matrix();
                    rlink->setPosition(v);
                    rlink->setRotation(q.w(),q.x(),q.y(),q.z());
                    T_var = rlink->T_visual;
                    rlink->type = urdf::Joint::FIXED;
                }
                
                rlink->objectName = ulink->name;
                creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);
                rlink->createEntity(scnMgr);
                
                std::cout << "Parsing link: " << ulink->name << std::endl;
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
        // get the position values from JointSharedPtr
        double x = _jptr->parent_to_joint_origin_transform.position.x;
        double y = _jptr->parent_to_joint_origin_transform.position.y;
        double z = _jptr->parent_to_joint_origin_transform.position.z;
        // set the position values to mRobotLink object.
        double qw,qx,qy,qz;
        _jptr->parent_to_joint_origin_transform.rotation.getQuaternion(qx,qy,qz,qw);
        Eigen::Quaterniond Q(qw,qx,qy,qz);
        Eigen::Vector3d v(x,y,z), t;
        Eigen::Affine3d T;
        
        _rlink->T_p_l.translation() = v;
        _rlink->T_p_l.linear() = Q.matrix();

        T = T_var.inverse() * _rlink->T_p_l * _rlink->T_visual;
        T_var = _rlink->T_visual;

        t = T.translation();
        Eigen::Quaterniond Qnew(T.rotation());
        Eigen::Vector4d q = Qnew.coeffs();

        _rlink->setPosition(t);
        _rlink->setRotation(q.w(),q.x(),q.y(),q.z());
        
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
        
        _rlink->T_visual.translation() = t;
        _rlink->T_visual.linear() = Q.matrix();

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
        Eigen::Vector3d vRot = Eigen::Vector3d::Zero();
        Eigen::Vector3d vTrans = Eigen::Vector3d::Zero();
        Eigen::Vector3d t;
        Eigen::Matrix3d R;

        Eigen::Affine3d T;

        // set T_var to be Identity;
        T_var.setIdentity();

        for (int i = 0; i < object_ptrs.size(); i++)
        {
            mRobotLinkPtr = object_ptrs[i];
            type = mRobotLinkPtr->type;

            if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS)
            {
                // vRot = mRobotLinkPtr->joint_variable * mRobotLinkPtr->axis;
                // mRobotLinkPtr->T_p_l.rotate(Eigen::AngleAxisd(mRobotLinkPtr->joint_variable,mRobotLinkPtr->axis));
                mRobotLinkPtr->T_p_l.linear() = Eigen::AngleAxisd(mRobotLinkPtr->joint_variable,mRobotLinkPtr->axis).matrix();
                // mRobotLinkPtr->T_p_l.translation() = vTrans;
            }
            else if (type == urdf::Joint::PRISMATIC)
            {
                vTrans = mRobotLinkPtr->joint_variable * mRobotLinkPtr->axis;
                mRobotLinkPtr->T_p_l.translation() = vTrans;
            }

            else if (type == urdf::Joint::FIXED)
            {
                T_var = mRobotLinkPtr->T_visual;
                // continue;
            }

            T = T_var.inverse() * mRobotLinkPtr->T_p_l * mRobotLinkPtr->T_visual;
            T_var = mRobotLinkPtr->T_visual;

            t = T.translation();
            R = T.linear();

            mRobotLinkPtr->setPosition(t);

            Eigen::Quaterniond Qnew(T.rotation());
            Eigen::Vector4d q = Qnew.coeffs();

            mRobotLinkPtr->setRotation(q.w(),q.x(), q.y(), q.z());
            // std::cout << mRobotLinkPtr->objectName << std::endl;
        }
        
        
    }

} // namespace mviz
