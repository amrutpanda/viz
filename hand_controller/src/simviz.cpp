#include <iostream>
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

#include "hc_redis_keys.h"

#include "ParallelRobot.h"
// std::string ROBOT_JOINT_KEY = "robot::q";


bool _simulation = true;

std::string robot_name = "hand_controller";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge_v1.1.urdf";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge.urdf";
std::string robot_file = "/home/merai/Downloads/005-HAND-CONTROLER-URDF/urdf/005-HAND-CONTROLER-URDF_v1.0.urdf";

std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}

inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);  // brings it to (-2π, 2π)
    if (angle < 0)
        angle += 2.0 * M_PI;                   // now in [0, 2π)
    return angle - M_PI;                       // final result in (-π, π]
}
void simulation(std::string& _robot_file);


RedisClient redis_client;
unsigned int nDof = 9;
Eigen::VectorXd _q,_q_prev, _delq;
Eigen::Vector3d xp;
// force sensor variables.


int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    // configure the _q;
    _q.resize(nDof);
    _q.setZero();

    _q_prev.resize(nDof);
    _q_prev.setZero();

    _delq.resize(nDof);
    _delq.setZero();

    Eigen::Quaterniond quat;
    quat.setIdentity();

    mviz::mVisualizer viz("simviz_hc");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);
    viz.getRobotObject(robot_name)->printRobotJointNames();
    
    viz.createSphere("sp1",0.02);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.setObjectPoseAndRotation("sp1",xp,quat);
        viz.updateRobotGraphics(robot_name,_q);
        viz.RenderOneFrame();
        if (!runloop)
            std::cout << "runloop flag is false. Exiting..." << std::endl;
        
        if (!_simulation)
        {
            _delq = _q - _q_prev;
            _q(2) =  _q(2) + (_delq(1) - _delq(nDof -2));
            _q(nDof - 1) = _q(nDof - 1) + (_delq(nDof -2) - _delq(1));
            _q_prev = _q;
        }

        // double _dq1 = 0.00002;
        // double _dq2 = 0.00002;
        // _q(1) = _q(1) + _dq1;
        // _q(6) = _q(6) + _dq2;
        // _q(7) = _q(7) +  (_dq1 - _dq2);
        // _q(2) = _q(2) + (_dq2 - _dq1);

        // _q(nDof -1) = (M_PI + (M_PI_2 + _q(1)) - ( M_PI + _q(nDof - 2)) - M_PI_2);
        // _q(2) = (M_PI + _q(nDof - 2)) -( M_PI_2 + _q(1));
    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    sim_thread.join();
    return 0;
}


void simulation(std::string& robot_file)
{
    ParallelRobot sim_model(robot_file,Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
    sim_model.setGravity(Eigen::Vector3d(0,0,-9.81*1));
    sim_model.updateKinematics();
    // set dof.
    nDof = sim_model.dof();

    Eigen::VectorXd command_torque;
    command_torque.resize(nDof);
    command_torque.setZero();


    Eigen::VectorXd _q_init, _dq_init;
    _q_init.resize(nDof);
    _q_init.setZero();

    _dq_init.resize(nDof - 2);
    _dq_init.setZero();

    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    sim_model.position(x,"gripper_link");

    // Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,nDof -2);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,nDof -2);

    Eigen::VectorXd G = Eigen::VectorXd::Zero(nDof - 2);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nDof - 2, nDof - 2);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(6);
    // Eigen::VectorXd F = Eigen::Vector3d::Zero();

    std::cout << "HI" << std::endl;
    _dq_init << 0.0000, 0.000102, 0.0000, 0.0000, 0.0000, 0.0000,0.0001;
    //  _dq_init << 0.000, 0.0000, 0.0000, -0.0000, 0.000,0.000;
    // _q_init << 0, 0, 0, -1.57, 0, 0, 0, 0.78;
    // _q_init(1) = -1.3;
    // sim_model._q = _q_init;
    std::cout << "Hoo" << std::endl;

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();

    std::cout << "Starting fsimulation loop" << std::endl;
    while (runloop && timer.WaitForNextLoop())
    {
        // sim_model._qr = sim_model._qr + _dq_init ;
        // sim_model._qr(1) = sim_model._qr(1) + 0.001;
        sim_model.UpdateParallelRobotKinematics();

        // sim_model.getJacobian(J,"gripper_link");

        // sim_model.getJacobian6D(J,"gripper_link");

        // x = x + J * _dq_init;

        // x = x + (J* _dq_init).head(3);

        // xp = x;
       
        // if (x(2) < 0.6)
        //     F(2) = 0.1;
        // else
        //     F.setZero();

        // command_torque = J.transpose() * F;
        // std::cout << "cmd torque: " << command_torque.transpose() << std::endl; 
        sim_model.getGravityVector(G);
        // sim_model.computeMassMatrix(M);
        // sim_model.ComputeMassMatrix(M);
        // sim_model.computeMassMatrixandGravityVector(M,G);
        std::cout << "G: \n" << G.transpose() << std::endl;
        Eigen::VectorXd _tor_damping = 0.0 * Eigen::VectorXd::Ones(nDof - 2);
        sim_model.forward_step(1.0* G, 0.01);

        _q = sim_model._q;
    }
    std::cout << "simulation" << std::endl;
    timer.printTimerHistory();
}