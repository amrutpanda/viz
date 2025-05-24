#include <iostream>
#include <dynamics_model.h>

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
        
        Eigen::VectorXd _current_position;
        Eigen::VectorXd _target_position;
        Eigen::VectorXd _current_velocity;
        Eigen::VectorXd _Kp;
        Eigen::VectorXd _Kv;
        Eigen::MatrixXd _Kp_mat;
        Eigen::MatrixXd _Kv_mat;

        Eigen::VectorXd h; // it will store the coriolis torque and gravity torque;
        double _goal_tolerance;
        int _nDof;
        bool velocity_saturation_flag = false;    
    
    };
} // namespace Primitives



