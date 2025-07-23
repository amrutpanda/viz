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
        _gravity = _g;
    }

    unsigned int DModel::linkId(const std::string& _link_name)
    {
        for (auto it : _rbdl_model->mBodyNameMap)
        {
            if (it.first == _link_name)
                return it.second;
        }
        throw std::runtime_error("Cannot find a link with name: " + _link_name);
    }

    unsigned int DModel::jointId(const std::string& _joint_name)
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

    void DModel::position(Eigen::Vector3d& _pos, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
        int linkInd = linkId(_link_name);
       
        _pos = CalcBodyToBaseCoordinates(*_rbdl_model,_q, linkInd, pos_in_link,false);

    }

    void DModel::positionInWorld(Eigen::Vector3d& _pos, const std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        Eigen::Affine3d _T_link_to_base;

        _T_link_to_base.setIdentity();

        std::string _link_name = link_name;
         int linkInd = linkId(_link_name);
       
        // _pos = CalcBaseToBodyCoordinates(*_rbdl_model,_q,linkInd,pos_in_link,false);
        _pos = CalcBodyToBaseCoordinates(*_rbdl_model,_q, linkInd, pos_in_link,false);
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

    void DModel::linearAcceleration6D(Eigen::Vector3d& _laccel, Eigen::Vector3d& _aaccel, std::string& link_name,
                    const Eigen::Vector3d& pos_in_link)
    {
        int linkInd = linkId(link_name);
        Eigen::VectorXd a_tmp(6);

        a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
        _aaccel = a_tmp.head(3); // first 3 components are rotational.
        _laccel = a_tmp.tail(3);
    }

    void DModel::linearAccelerationWorld6D(Eigen::Vector3d& _laccel, Eigen::Vector3d& _aaccel, std::string& link_name,
                    const Eigen::Vector3d& pos_in_link)
    {
        int linkInd = linkId(link_name);
        Eigen::VectorXd a_tmp(6);

        a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkInd,pos_in_link,false);
        _aaccel = _T_world.linear() * a_tmp.head(3); // first 3 components are rotational.
        _laccel = _T_world.linear() * a_tmp.tail(3);
    }

    void DModel::angularAcceleration(Eigen::Vector3d& _aaccel, std::string& link_name, const Eigen::Vector3d& pos_in_link)
    {
        // std::string _link_name = link_name;
        // int linkInd = linkId(_link_name);
        int linkInd = linkId(link_name);
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
    
    void DModel::computeIK(Eigen::VectorXd& _jposes, std::string& _link_name,
                                Eigen::Vector3d& target_pos,
                                Eigen::Vector3d pos_in_link)
    {
        _jposes.resize(_dof);
        std::string link_name = _link_name;
        InverseKinematicsConstraintSet CS;
        CS.AddPointConstraint(linkId(link_name),pos_in_link,target_pos);
        InverseKinematics(*_rbdl_model,_q,CS,_jposes);
    }

     void DModel::computeIK(Eigen::VectorXd& _jposes, std::string& _link_name,
                                Eigen::Vector3d& target_pos,
                                Eigen::Matrix3d& target_rot,        
                                Eigen::Vector3d pos_in_link)
    {
        _jposes.resize(_dof);
        std::string link_name = _link_name;
        InverseKinematicsConstraintSet CS;
        CS.AddFullConstraint(linkId(link_name),pos_in_link,target_pos,target_rot);
        InverseKinematics(*_rbdl_model,_q,CS,_jposes);
    }

   
    // Force sensor related methods.

    void DModel::setForceSensorLink(const std::string sensor_link, double _pMass, Eigen::Vector3d& _pCOM,
                                 Eigen::Vector3d sensor_pos_in_link, Eigen::Matrix3d R_link_sensor)
    {
        _force_sensor_link = sensor_link;
        _sensor_link_id = linkId(sensor_link);
        _sensor_pos_in_link = sensor_pos_in_link;
        _R_link_sensor = R_link_sensor;
        _payload_mass = _pMass;
        _payload_COM = _pCOM;
    }

    void DModel::getForceSensorOutput(Eigen::Vector3d& _force, Eigen::Vector3d& _moment)
    {
        Eigen::Matrix3d _R_world_sensor;
        rotation(_R_world_sensor,_force_sensor_link);
        // compute the force and moment due to gravity and remove it.
        Eigen::Vector3d _f_local_frame,_m_local_frame;
        _f_local_frame =  _payload_mass * _rbdl_model->gravity;
        _m_local_frame = _sensor_pos_in_link.cross(_f_local_frame);
         
        _force = _R_world_sensor.transpose() * (_sensed_force_raw + _f_local_frame);
        _moment = _R_world_sensor.transpose() * (_sensed_moment_raw + _m_local_frame);
        // round off the force to three digits after decimal.
        _force = _force.unaryExpr([](double x){return std::round(x*100)/1000;});
        _moment = _moment.unaryExpr([](double x){return std::round(x*1000)/1000;});
    }


     void orientationError(Eigen::Vector3d& orientation_error, Eigen::Matrix3d& _desired_orientation,
                                                                Eigen::Matrix3d& _current_orientation)
    {
        //check for valid rotation.
        Eigen::Matrix3d Q1 = _desired_orientation*_desired_orientation.transpose() - Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Q2 = _current_orientation*_current_orientation.transpose() - Eigen::Matrix3d::Identity();

        if (Q1.norm() >0.001 || Q2.norm() > 0.001)
        {
            std::cout << "Desired Orientation: " << _desired_orientation << std::endl;
            std::cout << "Current Orientation: " <<_current_orientation << std::endl;
            std::cout << "Q1 norm: " << Q1.norm() << std::endl;
            std::cout << "Q2 norm: " << Q2.norm() << std::endl;
            throw std::invalid_argument("Invalid rotation matrices. DModel orientation error\n");
            return; 
        }
        else
        {
            Eigen::Vector3d rc1 = _current_orientation.block<3,1>(0,0);
            Eigen::Vector3d rc2 = _current_orientation.block<3,1>(0,1);
            Eigen::Vector3d rc3 = _current_orientation.block<3,1>(0,2);
            Eigen::Vector3d rd1 = _desired_orientation.block<3,1>(0,0);
            Eigen::Vector3d rd2 = _desired_orientation.block<3,1>(0,1);
            Eigen::Vector3d rd3 = _desired_orientation.block<3,1>(0,2);
            orientation_error = (-1/2)*(rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
        }
        
    }

} // namespace Dynamics

