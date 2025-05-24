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

        _current_position = _robot_model->_q;
        _current_velocity = _robot_model->_dq;
    }   

    void JointTask::computeTorque(Eigen::VectorXd& _T)
    {

    }
} // namespace Primitives
