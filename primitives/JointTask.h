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
        double _goal_tolerance;
        int _nDof;
        bool velocity_saturation_flag = false;    
    
    };
} // namespace Primitives



