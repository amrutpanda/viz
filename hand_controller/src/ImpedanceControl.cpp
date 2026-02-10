#include "ImpedanceControl.hpp"

namespace control
{
    ImpedanceControl::ImpedanceControl(ParallelRobot* model, const std::string control_link_name,
                                                    const Eigen::VectorXd& control_pos_in_link,
                                                    const double loop_time = 0.001)
    {
        _model = model;
        _loop_time = loop_time;
        _control_link_name = control_link_name;
        _control_pos_in_link = control_pos_in_link;
        // initialize all the variable.
        n = model->_nr;

        _current_position = Eigen::Vector3d::Zero();
        _current_velocity = Eigen::Vector3d::Zero();
        _current_acceleration = Eigen::Vector3d::Zero();

        _desired_position = _current_position;
        _desired_velocity = _current_velocity;
        _desired_acceleration = _current_acceleration;
        
        _step_desired_position = _desired_position;
        _step_desired_velocity = _desired_velocity;
        _step_desired_acceleration = _desired_acceleration;
    #ifdef USING_ORIENTATION
        _current_orienation = Eigen::Matrix3d::Zero();
        _current_angular_velocity = Eigen::Vector3d::Zero();
        _current_angular_acceleration = Eigen::Vector3d::Zero();

        _desired_orienation = _current_orienation;
        _desired_angular_velocity = _current_angular_velocity;
        _desired_angular_acceleration = _current_angular_acceleration;
        
        _step_desired_orientation = _desired_orienation;
        _step_desired_angular_velocity = _desired_angular_velocity;
        _step_desired_angular_acceleration = _desired_angular_acceleration;
    #endif
        _prev_position = _current_position;
        _prev_velocity = _current_velocity;

        _F_imp = Eigen::VectorXd::Zero(6);
        F_ext = Eigen::VectorXd::Zero(6);
        _error = Eigen::VectorXd::Zero(6);
        _error_vel = _error;
        _error_acc = _error_acc;

        Kp_mat = Eigen::MatrixXd::Zero(6,6);
        Kd_mat = Eigen::MatrixXd::Zero(6,6);
        M_mat = Eigen::MatrixXd::Zero(6,6);

        M_modified = Eigen::MatrixXd::Zero(n,n);
        J = Eigen::MatrixXd::Zero(6,n);
        G = Eigen::VectorXd::Zero(n);

    }

    void ImpedanceControl::setKp(const Eigen::VectorXd& kp)
    {
        assert((kp.size() == 3, "Incorrect kp size"));
        Kp_mat = kp.asDiagonal();
    }

    void ImpedanceControl::setKd(const Eigen::VectorXd& kd)
    {
        assert((kd.size() == 3, "Incorrect kd size"));
        Kd_mat = kd.asDiagonal();   
    }

    void ImpedanceControl::setM(const Eigen::VectorXd& M)
    {
        assert((M.size() == 3, "Incorrect M size"));
        M_mat = M.asDiagonal();
    }

    void ImpedanceControl::setForce(const Eigen::VectorXd& f)
    {
        F_ext.head(3) = f;
    }

    void ImpedanceControl::updateModel()
    {
        _model->computeMassMatrixandGravityVector(M_modified,G);
        _model->getJacobian6D(J,_control_link_name,true,_control_pos_in_link);
    }

    void ImpedanceControl::computeTorques(Eigen::VectorXd& _command_torques)
    {
        if (_first_iteration)
        {
            t_prev = get_current_time_in_seconds();
            t_curr = get_current_time_in_seconds();
            t_diff = t_curr - t_prev;
        }
        else
        {
            t_curr = get_current_time_in_seconds();
            t_diff = t_curr - t_prev;
        }

        _model->position(_current_position,_control_link_name,_control_pos_in_link);
        _model->linearVelocity(_current_velocity,_control_link_name,_control_pos_in_link);
        _model->linearAcceleration(_current_acceleration,_control_link_name,_control_pos_in_link);

    #ifdef USING_ORIENTATION
        _model->rotation(_current_orienation,_control_link_name);
        _model->angularVelocity(_current_angular_velocity,_control_link_name,_control_pos_in_link);
        _model->angularAcceleration(_current_angular_acceleration,_control_link_name,_control_pos_in_link);

        _model->orientationError(orientation_error,_desired_orienation,_current_orienation);
        _error.tail(3) = orientation_error;
        _error.tail(3) = _desired_angular_velocity - _current_angular_velocity;
        _error_acc.tail(3) = _desired_angular_acceleration - _current_acceleration;
    #endif

        _error.head(3) = _desired_position - _current_position;
        _error_vel.head(3) = _desired_velocity - _current_velocity;
        _error_acc.head(3) = _desired_acceleration - _current_acceleration;

    // #ifdef USING_ORIENTATION
    //     _model->orientationError(orientation_error,_desired_orienation,_current_orienation);
    //     _error.tail(3) = orientation_error;
    //     _error.tail(3) = _desired_angular_velocity - _current_angular_velocity;
    //     _error_acc.tail(3) = _desired_angular_acceleration - _current_acceleration;
    // #endif

        _F_imp = M_mat*_error + Kd_mat * _error_vel + Kp_mat * _error_acc;
        _command_torques = J.transpose()*(_F_imp + F_ext);
        // set external force to zero, which will be update in every iteration.
        F_ext.setZero();
    }
} // namespace control
