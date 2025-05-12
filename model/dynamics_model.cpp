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

    void DModel::transformation(Eigen::Vector3d& _pos, Eigen::Matrix3d& _rot,
        const std::string& link_name,
        const Vector3d& pos_in_link = Eigen::Vector3d::Zero(),
        const Matrix3d& rot_in_link = Eigen::Matrix3d::Identity(),
        const std::string& base_frame = "")
    {
        Eigen::Affine3d _T_link_to_base;
        Eigen::Affine3d _T_bframe_to_base;
        Eigen::Affine3d _T;

        _T_link_to_base.setIdentity();
        _T_bframe_to_base.setIdentity();

        std::string _link_name = link_name;
        std::string _base = base_frame;

        int linkInd = linkId(_link_name);
        int baseInd = linkId(_link_name);
        
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

        // _T = _T_bframe_to_base.inverse() * 
        
    }

} // namespace Dynamics

