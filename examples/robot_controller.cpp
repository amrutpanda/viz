#include <iostream>
#include <redisclient.h>
#include <LoopTimer.h>
#include <dynamics_model.h>

std::string ROBOT_JOINT_POSITION_KEY = "robot::q";
std::string ROBOT_JOINT_VELOCITY_KEY = "robot::dq";
std::string ROBOT_JOINT_TORQUE_KEY = "robot::command_torque";

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/amrut/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";


bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_qdot;
Eigen::VectorXd _command_torques;


int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    Eigen::Vector3d _bpose(0,0,0);
    Eigen::Quaterniond _brot;
    _brot.setIdentity();

    Dynamics::DModel* robot_model = new Dynamics::DModel(robot_file,_bpose,_brot);
    // resize the vectors.
    int n = 0;
    _q.resize(n);
    _qdot.resize(n);
    _command_torques.resize(n);

    RedisClient redis_client;
    redis_client.connect();
    int r1 = redis_client.createEigenReadCallback(ROBOT_JOINT_POSITION_KEY,_q);
    int r2 = redis_client.createEigenReadCallback(ROBOT_JOINT_VELOCITY_KEY,_qdot);
    int w1 = redis_client.createEigenWriteCallback(ROBOT_JOINT_TORQUE_KEY,_command_torques);

    Eigen::VectorXd _gravity(n);

    LoopTimer timer;
    timer.setLoopFrequency(200);
    timer.InitializeTimer();

    while (runloop)
    {
        redis_client.executeAllReadCallbacks();
        
    }
    


    return 0;
}

