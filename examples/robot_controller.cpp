#include <iostream>
#include <redisclient.h>
#include <LoopTimer.h>
#include <dynamics_model.h>
#include <redis_keys.h>

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";


bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_dq;
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
    _q.resize(nDof);
    _dq.resize(nDof);
    _command_torques.resize(nDof);

    Eigen::VectorXd pos(6);
    pos.setZero();
    pos(0) = 0.1;

    RedisClient redis_client;
    redis_client.connect();
    int r1 = redis_client.createEigenReadCallback(ROBOT_JOINT_POSITION_KEY,_q);
    int r2 = redis_client.createEigenReadCallback(ROBOT_JOINT_VELOCITY_KEY,_dq);
    int w1 = redis_client.createEigenWriteCallback(ROBOT_JOINT_TORQUE_KEY,_command_torques);

    Eigen::VectorXd _gravity(n);
    Eigen::VectorXd b(nDof), g(nDof);
    Eigen::MatrixXd M(nDof, nDof);

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();

    // set the controller key.
    std::string _controller_running = "1";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    while (runloop && timer.WaitForNextLoop())
    {
        redis_client.executeAllReadCallbacks();
        robot_model->_q = _q;
        robot_model->_dq = _dq;
        robot_model->updateModel();

        robot_model->gravityVector(g);
        // _command_torques = - robot_model->_M *(50 *(_q - pos) + 2* _dq) + g;
        _command_torques = g;
    
        // std::cout << "command torque: \n" << _command_torques << std::endl;
        redis_client.executeAllWriteCallbacks();
    }
    _controller_running = "0";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    std::cout << "Controller history: " << std::endl;
    timer.printTimerHistory();
    return 0;
}

