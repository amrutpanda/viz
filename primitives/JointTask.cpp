#include <JointTask.h>

namespace Primitives
{
    JointTask::JointTask(Dynamics::DModel* _model)
    {
        _robot_model = _model;
        _nDof = _robot_model->dof();

        _current_position.resize(_nDof);
        _target_position.resize(_nDof);
        _current_velocity.resize(_nDof);
        _Kp.resize(_nDof);
        _Kv.resize(_nDof);
        h.resize(_nDof);

        // set default _kp and _kv;
        _Kp = 200 * Eigen::VectorXd::Ones(_nDof);
        _Kv = 28 * Eigen::VectorXd::Ones(_nDof);

        _Kp_mat = _Kp.asDiagonal();
        _Kv_mat = _Kv.asDiagonal();

        _current_position = _robot_model->_q;
        _current_velocity = _robot_model->_dq;

        velocity_saturation_flag = false; // If set to true, saturation velocity need to be set.

        // workspace limit setting.
        _r_lim = 0;
        _rot_lim = 0;

        _compute_ws_pos_lim = false;
        _compute_ws_rot_lim = false;

    }   

    void JointTask::computeTorque(Eigen::VectorXd& _T)
    {
        // need to update the robot model before calling this
        // function for correct torque computation.
        _current_position = _robot_model->_q;
        _current_velocity = _robot_model->_dq;
        // _robot_model->coriolisPlusGravityForces(h);
        _robot_model->gravityVector(h);
        // _T = _robot_model->_M( -_Kp_mat* (_current_position - _target_position) 
        //                         - _Kv_mat*(_current_velocity)) + h;
        if (velocity_saturation_flag)
        {
            // TO-DO: need to implement velocity saturation computation.
        }
        else
        {
            // if (!_use_interpolation)
            // {    
            //     _T = _robot_model->_M*( -_Kp_mat* (_current_position - _target_position) 
            //                         - _Kv_mat*(_current_velocity)) + h;
            // }
            // else
            // {
                
            // }
            
            _T = _robot_model->_M*( -_Kp_mat* (_current_position - _target_position) 
                                        - _Kv_mat*(_current_velocity)) + h;
        }

        
    }

    bool JointTask::HasReachedTarget()
    {
        double _dNorm = (_target_position - _current_position).norm();
        if (_dNorm <= _goal_tolerance)
            return true;
        return false;
    }

    void JointTask::reInitializeTask()
    {
        _current_position = _robot_model->_q;
        _current_velocity = _robot_model->_dq;

        _target_position = _current_position;
    }

    void JointTask::setKp(Eigen::VectorXd& _kp)
    {
       assert(_kp.size() == _nDof);
       _Kp_mat = _kp.asDiagonal();
    }

    void JointTask::setKv(Eigen::VectorXd& _kv)
    {
        assert(_kv.size() == _nDof);
        _Kv_mat = _kv.asDiagonal();
    }

    void JointTask::setWorkspaceCenter(Eigen::Vector3d& _robot_ws_center, Eigen::Matrix3d& _robot_ws_rot_center)
    {
        _workspace_center = _robot_ws_center;
        _workspace_rot_center = _robot_ws_rot_center;
    }

    void JointTask::setWorkspaceLimit(const double _rlimit, const double _rotlimit)
    {
        // set position limit. The limit is like a sphere center around ws_center.
        if (_rlimit <= 0)
        {
            _compute_ws_pos_lim = false;
        }
        else
        {
            _r_lim = _rlimit;
            _compute_ws_pos_lim = true;
        }
        
        // if norm of the _rotLimit vector is zero, workspace limit won't be evaluated.
        if (_rotlimit == 0)
        {
            _compute_ws_rot_lim = false;
        }
        else
        {
            _compute_ws_rot_lim = true;
            _rot_lim = _rotlimit;
        }
        
    }

    void JointTask::enforceWorkspaceLimit(Eigen::Vector3d& _robot_ee_pos, Eigen::Matrix3d& _robot_ee_rot)
    {
        if (_compute_ws_pos_lim)
        {
            // find the current radius from ws center.
            auto _diff = _robot_ee_pos - _workspace_center;
            double _r_curr = _diff.norm();
            if (_r_curr > _r_lim)
            {
                // std::cout << "robot proxy: before: " << _robot_ee_pos.transpose() << std::endl;
                _robot_ee_pos = _workspace_center + _r_lim *_diff/ _r_curr;
                // std::cout << "robot proxy: after: " << _robot_ee_pos.transpose() << std::endl;
            }
        }

        if (_compute_ws_rot_lim)
        {
            auto _diff_rot = _robot_ee_rot * _workspace_rot_center.transpose();
            Eigen::AngleAxisd aa(_diff_rot);
            aa.angle() = std::clamp(aa.angle(), -_rot_lim, _rot_lim);
            _robot_ee_rot = aa.toRotationMatrix() * _workspace_rot_center;
        }                   
    }
} // namespace Primitives
