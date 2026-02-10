#include <iostream>
// #include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>
#include "hc_redis_keys.h"

#include "ParallelRobot.h"

// std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "hand_controller";
std::string control_link_name = "gripper_link";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller.urdf";
std::string robot_file = "/home/merai/Downloads/000-HAND-CONTROLER-URDF_v1.1/urdf/000-HAND-CONTROLER-URDF_v1.1.urdf";
// std::string robot_file = "/home/merai/Downloads/000-HAND-CONTROLER-URDF_v1.2/urdf/000-HAND-CONTROLER-URDF_NO_GIMBAL_v1.2.urdf";
// std::string robot_file = "/home/merai/Downloads/000-HAND-CONTROLER-URDF_v1.2/urdf/000-HAND-CONTROLER-URDF_v1.2.1.urdf";

std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}

inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);  // brings it to (-2π, 2π)
    if (angle < 0)
        angle += 2.0 * M_PI;                   // now in [0, 2π)
    return angle - M_PI;                       // final result in (-π, π]
}

Eigen::VectorXd _q, _dq, command_torques;

int main(int argc, char const *argv[])
{

    signal(SIGINT,sighandler);

    RedisClient redis_client;
    redis_client.connect();
    

    ParallelRobot sim_model(robot_file,Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
    sim_model.setGravity(Eigen::Vector3d(0,0,-9.81*1));
    sim_model.updateKinematics();

    int nDof = sim_model.dof();

    // set redis client.
    // create redis client callbacks.
    // redis_client.createEigenReadCallback(HC_JOINT_POSITION_KEY,_q);
    // redis_client.createEigenReadCallback(HC_JOINT_VELOCITY_KEY,_dq);
    // redis_client.createEigenWriteCallback(HC_JOINT_TORQUE_KEY,command_torques);

    Eigen::VectorXd q_init;
    q_init.resize(nDof);
    q_init.setZero();

    std::cout << "C: \n" << sim_model.C << std::endl; 

    Eigen::MatrixXd M,J;

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    std::cout << "Starting simf loop" << std::endl;

    // redis_client.set(HC_CONTROLLER_RUNNING_KEY,"1");
    while (runloop & timer.WaitForNextLoop())
    {
        // redis_client.executeAllReadCallbacks();



        // redis_client.executeAllWriteCallbacks();
    }
    std::cout << "exiting loop" << std::endl;
    // redis_client.setEigenMatrix(HC_JOINT_POSITION_KEY, q_init);
    // redis_client.set(HC_CONTROLLER_RUNNING_KEY,"0");
    timer.printTimerHistory();

    return 0;
}