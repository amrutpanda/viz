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
} // namespace Primitives
