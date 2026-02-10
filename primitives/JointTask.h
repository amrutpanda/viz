#ifndef _PRIMITIVE_H
#define _PRIMITIVE_H
#include <iostream>
#include <dynamics_model.h>
#include <ruckig/ruckig.hpp>

namespace Primitives
{ 
    class JointTask
    {
    private:
        Dynamics::DModel* _robot_model;
        Eigen::Vector3d _workspace_center;
        Eigen::Matrix3d _workspace_rot_center;
        double _r_lim;
        double _rot_lim; // in radian
        bool _compute_ws_pos_lim;
        bool _compute_ws_rot_lim;

    public:
        JointTask(Dynamics::DModel* );
        ~JointTask();
        
        void computeTorque(Eigen::VectorXd& _T);
        bool HasReachedTarget();
        void reInitializeTask();
        void setKp(Eigen::VectorXd& _kp);
        void setKv(Eigen::VectorXd& _kv);

        void setWorkspaceCenter(Eigen::Vector3d& _robot_ws_center, Eigen::Matrix3d& _robot_ws_rot_center);
        /**
         * Limits the workspace of the robot to a sphere of given radius with current pos the center.
         * Rotation limits is the rotation limit in both +ve and -ve axis for all the axis.
        */
        void setWorkspaceLimit(const double _rlimit, const double _rotlimit);
        void enforceWorkspaceLimit(Eigen::Vector3d& _robot_ee_pos, Eigen::Matrix3d& _robot_ee_rot);
        
        Eigen::VectorXd _current_position;
        Eigen::VectorXd _target_position;
        Eigen::VectorXd _current_velocity;
        Eigen::VectorXd _Kp;
        Eigen::VectorXd _Kv;
        Eigen::MatrixXd _Kp_mat;
        Eigen::MatrixXd _Kv_mat;

        Eigen::VectorXd h; // it will store the coriolis torque and gravity torque;
        int _nDof;
        bool _use_interpolation = false;
        bool velocity_saturation_flag = false;
        double _saturation_velocity = 0.0; 
        double _goal_tolerance = 0.01; 
    
    };
} // namespace Primitives

#endif

