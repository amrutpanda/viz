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
    public:
        JointTask(Dynamics::DModel* );
        ~JointTask();
        
        void computeTorque(Eigen::VectorXd& _T);
        bool HasReachedTarget();
        void reInitializeTask();
        void setKp(Eigen::VectorXd& _kp);
        void setKv(Eigen::VectorXd& _kv);
        
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

