#ifndef _IMPEDANCE_CONTROL_H
#define _IMPEDANCE_CONTROL_H

#include <iostream>
#include "ParallelRobot.h"

#define USING_ORIENTATION
#define USING_MASS_MATRIX

namespace control
{
    class ImpedanceControl
    {
    private:
        double _loop_time;
        int n;            // num of joints to control i.e dof
        // ParallelRobot object pointer
        ParallelRobot* _model;
        
        Eigen::Vector3d _step_desired_position;
        Eigen::Vector3d _step_desired_velocity;
        Eigen::Vector3d _step_desired_acceleration;

#ifdef USING_ORIENTATION
        Eigen::Matrix3d _step_desired_orientation;
        Eigen::Vector3d _step_desired_angular_velocity;
        Eigen::Vector3d _step_desired_angular_acceleration; 
#endif

        Eigen::VectorXd _error;
        Eigen::VectorXd _error_vel;
        Eigen::VectorXd _error_acc;
#ifdef USING_ORIENTATION
        Eigen::Vector3d orientation_error;
#endif
        Eigen::Vector3d _prev_position;
        Eigen::Vector3d _prev_velocity;

        Eigen::MatrixXd Kp_mat;
        Eigen::MatrixXd Kd_mat;
        Eigen::MatrixXd M_mat;

        // container to store force value;
        Eigen::VectorXd _F_imp;
        Eigen::VectorXd F_ext;

        // set time variables.
        timespec t;
        double t_curr, t_prev, t_diff;

        bool _first_iteration = true;

        double get_current_time_in_seconds()
        {
            clock_gettime(CLOCK_MONOTONIC,&t);
            return t.tv_sec + t.tv_nsec*1e-9;
        }

    public:
        ImpedanceControl(ParallelRobot* _model, const std::string control_link_name,
                                            const Eigen::VectorXd& control_pos_in_link,
                                            const double loop_time );
        ~ImpedanceControl() {};
        void setKp(const Eigen::VectorXd& kp);
        void setKd(const Eigen::VectorXd& kd);
        void setM(const Eigen::VectorXd& M);
        void setForce(const Eigen::VectorXd& f);
        void computeTorques(Eigen::VectorXd& _command_torques);
        void updateModel();

        std::string _control_link_name;
        Eigen::Vector3d _control_pos_in_link;
        Eigen::Matrix3d _control_rot_in_link;

        Eigen::Vector3d _current_position;
        Eigen::Vector3d _current_velocity;
        Eigen::Vector3d _current_acceleration;
    #ifdef USING_ORIENTATION
        Eigen::Matrix3d _current_orienation;
        Eigen::Vector3d _current_angular_velocity;
        Eigen::Vector3d _current_angular_acceleration;
    #endif

        Eigen::Vector3d _desired_position;
        Eigen::Vector3d _desired_velocity;
        Eigen::Vector3d _desired_acceleration;

    #ifdef USING_ORIENTATION
        Eigen::Matrix3d _desired_orienation;
        Eigen::Vector3d _desired_angular_velocity;
        Eigen::Vector3d _desired_angular_acceleration;
    #endif
        // robot model specific/
        Eigen::MatrixXd M_modified;
        Eigen::MatrixXd J;
        Eigen::VectorXd G;

    };
} // namespace control

#endif