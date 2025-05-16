#include <dynamics_model.h>

namespace Dynamics
{
    DModel::DModel(std::string _robot_file, Eigen::Vector3d& _bpose,
                    Eigen::Quaterniond& _brot, bool floating_base ,bool _verbose)
    {
        _rbdl_model = new Model();
        bool result = URDFReadFromFile(_robot_file.c_str(),_rbdl_model,floating_base,_verbose);
        if (!result)
            throw std::runtime_error("Error while loading URDF file.\n");
        _dof = _rbdl_model->dof_count;
        _q_size = _rbdl_model->q_size;
        _q.resize(_dof);
        _dq.resize(_dof);
        _ddq.resize(_dof);

        _M.resize(_dof,_dof);
        _M_inv.resize(_dof,_dof);

        _q.setZero();
        _dq.setZero();
        _ddq.setZero();

        _M.setIdentity();
        _M_inv.setIdentity();

        _rbdl_model->gravity = Eigen::Vector3d(0,0,-9.81);

        // setup the world transformation matrix.
        _T_world.setIdentity();
        _T_world.translation() = _bpose;
        _T_world.linear() = _brot.toRotationMatrix();
    }

    DModel::~DModel()
    {
        delete _rbdl_model;
        _rbdl_model = NULL;
    }

    void DModel::setGravity(Eigen::Vector3d& _g)
    {
        _rbdl_model->gravity = _g;
    }

    unsigned int DModel::linkId(std::string& _link_name)
    {
        for (auto it : _rbdl_model->mBodyNameMap)
        {
            if (it.first == _link_name)
                return it.second;
        }
        
        throw std::runtime_error("Cannot find a link with name: " + _link_name);
    }

    unsigned int DModel::jointId(std::string& _joint_name)
    {
        
        for ( auto it : _rbdl_model->mBodyNameMap )
        {
            /* code */
        }
        return 0;   
    }

    int DModel::dof()
    {
        return _dof;
    }

    int DModel::q_size()
    {
        return _q_size;
    }

    void DModel::updateKinematics()
    {
        UpdateKinematicsCustom(*_rbdl_model, &_q,&_dq,&_ddq);
    }

    void DModel::updateDynamics()
    {
        if (_M.rows() != _dof || _M.cols() != _dof)
            _M.setZero(_dof,_dof);
        CompositeRigidBodyAlgorithm(*_rbdl_model,_q,_M,false); // set updateKinematics as false
        updateInverseInertia();
    }
    
    void DModel::updateInverseInertia()
    {
        _M_inv = _M.inverse();
    }

    void DModel::updateModel()
    {
        updateKinematics();
        updateDynamics();
        
    }

    void DModel::transformation(Eigen::Vector3d& _pos, Eigen::Matrix3d& _rot, const std::string& link_name,
                                const Vector3d& pos_in_link, const Matrix3d& rot_in_link, const std::string& base_frame )
    {
        Eigen::Affine3d _T_link_to_base;
        Eigen::Affine3d _T_bframe_to_base;
        Eigen::Affine3d _T;

        _T_link_to_base.setIdentity();
        _T_bframe_to_base.setIdentity();

        std::string _link_name = link_name;
        std::string _base = base_frame;

        int linkInd = linkId(_link_name);
        int baseInd = linkId(_base);
        
        if (base_frame.empty())
        {
            _T_link_to_base.translation() = CalcBaseToBodyCoordinates(*_rbdl_model,_q,linkInd,pos_in_link,false);
            _T_link_to_base.linear() = CalcBodyWorldOrientation(*_rbdl_model,_q,linkInd,false).transpose() * rot_in_link;
        }
        else
        {
            _T_link_to_base.translation() = CalcBaseToBodyCoordinates(*_rbdl_model,_q,linkInd,pos_in_link,false);
            _T_link_to_base.linear() = CalcBodyWorldOrientation(*_rbdl_model,_q,linkInd,false).transpose() * rot_in_link;

            _T_bframe_to_base.translation() = CalcBaseToBodyCoordinates(*_rbdl_model,_q,baseInd,
                                                                Eigen::Vector3d::Zero(),false);
            _T_bframe_to_base.linear() = CalcBodyWorldOrientation(*_rbdl_model,_q,baseInd,false).transpose() * rot_in_link;
        }

        _T = _T_bframe_to_base.inverse() * _T_link_to_base;
        _pos = _T.translation();
        _rot = _T.linear();
        
    }

    void DModel::position(Eigen::Vector3d& _pos, std::string& link_name, const Eigen::Vector3d& pos_in_link)

    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
       
        _pos = CalcBaseToBodyCoordinates(*_rbdl_model,_q,linkInd,pos_in_link,false);

    }

    void DModel::positionInWorld(Eigen::Vector3d& _pos, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
       
        _pos = CalcBaseToBodyCoordinates(*_rbdl_model,_q,linkInd,pos_in_link,false);
        _pos = _T_world * _pos;
    }

    void DModel::rotation(Eigen::Matrix3d& _rot, std::string& link_name, const Eigen::Matrix3d& rot_in_link)
    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
       
        _rot = CalcBodyWorldOrientation(*_rbdl_model,_q,linkInd,false).transpose();
        _rot = _rot * rot_in_link;
    }

    void DModel::rotationInWorld(Eigen::Matrix3d& _rot, std::string& link_name, const Eigen::Matrix3d& rot_in_link)
    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
       
        _rot = CalcBodyWorldOrientation(*_rbdl_model,_q,linkInd,false).transpose();
        _rot = _T_world.linear() * _rot * rot_in_link;
    }


    void DModel::linearVelocity(Eigen::Vector3d& _vel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);

        _vel = CalcPointVelocity(*_rbdl_model,_q,_dq,linkInd,pos_in_link,false);
    }

    void DModel::linearVelocityWorld(Eigen::Vector3d& _vel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);

        _vel = _T_world.linear() * CalcPointVelocity(*_rbdl_model,_q,_dq,linkInd,pos_in_link,false);
    }

    void DModel::angularVelocity(Eigen::Vector3d& _avel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
        Eigen::VectorXd v_tmp(6);

        v_tmp = CalcPointVelocity6D(*_rbdl_model,_q,_dq,linkInd,pos_in_link,false);
        _avel = v_tmp.head(3);
    }

    void DModel::angularVelocityWorld(Eigen::Vector3d& _avel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
        Eigen::VectorXd v_tmp(6);

        v_tmp = CalcPointVelocity6D(*_rbdl_model,_q,_dq,linkInd,pos_in_link,false);
        _avel = _T_world.linear() * v_tmp.head(3);
    }

    void DModel::linearAcceleration(Eigen::Vector3d& _accel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);

        _accel = CalcPointAcceleration(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
    }

    void DModel::linearAccelerationWorld(Eigen::Vector3d& _accel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);

        _accel = _T_world.linear() * CalcPointAcceleration(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
    }

    void DModel::angularAcceleration(Eigen::Vector3d& _aaccel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
        Eigen::VectorXd a_tmp(6);

        a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
        _aaccel = a_tmp.head(3);
    }

    void DModel::angularAccelerationWorld(Eigen::Vector3d& _aaccel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
        Eigen::VectorXd a_tmp(6);

        a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
        _aaccel = _T_world.linear() * a_tmp.head(3);
    }

    void DModel::gravityVector(Eigen::VectorXd& g)
    {
        Eigen::Vector3d _gravity = _rbdl_model->gravity;

        if (g.size() != _dof)
            g.resize(_dof);
        g.setZero();

        int body_id = 0;
        Eigen::MatrixXd Jv;
        Jv.resize(3,_dof);
        Jv.setZero();
        for (auto it : _rbdl_model->mBodies)
        {
            double mass = it.mMass;
            CalcPointJacobian(*_rbdl_model,_q,body_id,it.mCenterOfMass,Jv,false);
            g = g + Jv.transpose() * _T_world.linear().transpose() *(- mass * _gravity);
            body_id++;
        }
    }

    void DModel::coriolisForces(Eigen::VectorXd& c)
    {
        NonlinearEffects(*_rbdl_model,_q,_dq,c);
        Eigen::VectorXd g(_dof);
        g.setZero();
        gravityVector(g);
        c -= g;
    }

    void DModel::coriolisPlusGravityForces(Eigen::VectorXd& h)
    {
        NonlinearEffects(*_rbdl_model,_q,_dq,h);
    }
    /**
     * @brief 6D jacobian computation. Jacobian computed from base frame.
    */
    void DModel::J_0(Eigen::MatrixXd& J, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        Eigen::MatrixXd J_tmp;
        J_tmp.resize(6,_dof);

        CalcPointJacobian6D(*_rbdl_model,_q,linkId(_link_name),pos_in_link,J_tmp,false);
        // RBDL model computes Jw in first 3 rows and Jv on the bottom part. Swapping needed.
        J << J_tmp.block(3,0,6,_dof) , J_tmp.block(0,0,3,_dof);
    }

    /**
     * @brief 6D jacobian computation. Jacobian computed from World frame.
    */
   void DModel::J_0_World_Frame(Eigen::MatrixXd& J, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
   {
       std::string _link_name = link_name;
       Eigen::MatrixXd J_tmp;
       J_tmp.resize(6,_dof);

       CalcPointJacobian6D(*_rbdl_model,_q,linkId(_link_name),pos_in_link,J_tmp,false);
       // RBDL model computes Jw in first 3 rows and Jv on the bottom part. Swapping needed.
       J << _T_world.linear()* J_tmp.block(3,0,6,_dof) , _T_world.linear() * J_tmp.block(0,0,3,_dof);
   }

   /**
     * @brief 6D jacobian computation. Jacobian computed from Local frame.
    */
   void DModel::J_0_Local_Frame(Eigen::MatrixXd& J, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
   {
       std::string _link_name = link_name;
       Eigen::MatrixXd J_tmp;
       J_tmp.resize(6,_dof);
       J.resize(6,_dof);

       CalcPointJacobian6D(*_rbdl_model,_q,linkId(_link_name),pos_in_link,J_tmp,false);
       // RBDL model computes Jw in first 3 rows and Jv on the bottom part. Swapping needed.
       J << _T_world.linear()* J_tmp.block(3,0,6,_dof) , _T_world.linear() * J_tmp.block(0,0,3,_dof);
   }



   void DModel::J_v(Eigen::MatrixXd& Jv, const std::string& link_name,
                                    const Eigen::Vector3d& pos_in_link)
   {
        std::string _link_name = link_name;
        Jv.resize(3,_dof);
        CalcPointJacobian(*_rbdl_model,_q,linkId(_link_name),pos_in_link,Jv,false);
   }

   void DModel::J_v_World_Frame(Eigen::MatrixXd& Jv, const std::string& link_name,
                                        const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        Jv.resize(3,_dof);
        Eigen::MatrixXd Jv_tmp;
        
        CalcPointJacobian(*_rbdl_model,_q,linkId(_link_name),pos_in_link,Jv,false);
        Jv = _T_world.linear() * Jv;
    }

    void DModel::J_w(Eigen::MatrixXd& Jw, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        Jw.resize(3,_dof);
        Eigen::MatrixXd Jv_tmp;
        Jv_tmp.resize(6,_dof);

        CalcPointJacobian6D(*_rbdl_model,_q,linkId(_link_name),pos_in_link,Jv_tmp,false);
        Jw = Jv_tmp.block(0,0,3,_dof);
    }

    void DModel::J_w_World_Frame(Eigen::MatrixXd& Jw, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        std::string _link_name = link_name;
        Jw.resize(3,_dof);
        Eigen::MatrixXd Jv_tmp;
        Jv_tmp.resize(6,_dof);

        CalcPointJacobian6D(*_rbdl_model,_q,linkId(_link_name),pos_in_link,Jv_tmp,false);
        Jw = _T_world.linear() * Jv_tmp.block(0,0,3,_dof);
    }
    
} // namespace Dynamics

