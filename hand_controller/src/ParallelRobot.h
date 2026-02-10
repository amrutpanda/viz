#ifndef _CLOSE_KINEMATIC_CHAIN_ROBOT_H
#define _CLOSE_KINEMATIC_CHAIN_ROBOT_H

#include <dynamics_model.h>
#include <Eigen/QR>
// #include <rbdl-casadi/rbdl/rbdl.h>
// #include <LoopTimer.hpp>

#include <time.h>

using namespace Dynamics;
    
class ParallelRobot : public DModel
{
private:
    // contains which indexes of the joint angle vector to keep.
    // Useful for computing reduced jacobian, mass matrix and gravity vector.
    std::vector<int> idxs;
    Eigen::VectorXd _q_prev;
    Eigen::VectorXd _dq_prev;
    Eigen::VectorXd _ddq_prev;

    // some private variables for computation.
    Eigen::MatrixXd _J;
    Eigen::MatrixXd _J_tmp;
    Eigen::MatrixXd _Jv;
    Eigen::MatrixXd _Jv_tmp;
    Eigen::MatrixXd _J_notSwapped;
    Eigen::MatrixXd Is;

    // Eigen::MatrixXd C;
    inline double get_current_time()
    {
        clock_gettime(CLOCK_MONOTONIC,&t);
        return (t.tv_sec + t.tv_nsec * 1e-9);
    }
    
public:
    ParallelRobot(const std::string& robot_file, const Eigen::Vector3d& , const Eigen::Quaterniond& ,
            bool floating_base = false, bool _verbose = true);
    ~ParallelRobot() {};

    void computeMappingMatrix(Eigen::MatrixXd& mat);

    void computeMassMatrix(Eigen::MatrixXd& M );

    void ComputeMassMatrix(Eigen::MatrixXd& M);
    
    void getJacobian(Eigen::MatrixXd& J, const std::string& link_name, bool reduced = true,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void getJacobian6D(Eigen::MatrixXd& J, const std::string& link_name, bool reduced = true,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void getGravityVector(Eigen::VectorXd& _g);

    void computeMassMatrixandGravityVector(Eigen::MatrixXd& M, Eigen::VectorXd& G);

    void UpdateParallelRobotModel();
    void UpdateParallelRobotKinematics();

    void forward_step(Eigen::VectorXd _tor, double _tstep = 0.001);

    void getRc();
    void setFc();
    void assignPassiveJoint();

    int _nr;
    unsigned int _nDof;
    Eigen::VectorXd _qr;
    Eigen::VectorXd _dqr;
    Eigen::VectorXd _dqr_prev;
    Eigen::VectorXd _ddqr;
    
    double t_curr,t_prev;
    timespec t;
    bool _first_iteration;

    Eigen::MatrixXd J_c;
    Eigen::MatrixXd M_c;

    Eigen::MatrixXd C;
};


#endif