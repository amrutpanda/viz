#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <redisclient.h>
#include <LoopTimer.h>

std::string ROBOT_JOINT_POSITION_KEY = "robot::q";
std::string ROBOT_JOINT_VELOCITY_KEY = "robot::dq";
std::string ROBOT_JOINT_TORQUE_KEY = "robot::torque";

std::string robot_name = "cr12_robot";
std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";

bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_qdot;
Eigen::VectorXd _command_torques;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;
using namespace RigidBodyDynamics::Math;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    Model* model = new Model();
    // load urdf.
    bool res = URDFReadFromFile(robot_file.c_str(),model,false,true);
    if (!res)
        throw std::runtime_error("Unable to Load the URDF file.\n");
    
    // resize the vectors.
    int n = model->q_size;
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
        
        model->gravity(_gravity);
        
    }
    


    return 0;
}

