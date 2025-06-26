#include <iostream>
#include <redisclient.h>
#include <LoopTimer.h>
#include <dynamics_model.h>
#include <redis_keys.h>
#include <JointTask.h>

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
// force sensor variables.
Eigen::Vector3d _force, _moment;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    Eigen::Vector3d _bpose(0,0,0);
    Eigen::Quaterniond _brot;
    _brot.setIdentity();
    Dynamics::DModel* robot_model = new Dynamics::DModel(robot_file,_bpose,_brot,
                                                          false,false);
    // resize the vectors.
    int n = 0;
    _q.resize(nDof);
    _dq.resize(nDof);
    _command_torques.resize(nDof);
    _command_torques.setZero();

    Eigen::VectorXd pos(6);
    pos.setZero();
    // pos(0) = 0.1;
    pos << 0.1,0.4,-2.0,-0.32,0,0;

    RedisClient redis_client;
    redis_client.connect();
    redis_client.createEigenReadCallback(ROBOT_JOINT_POSITION_KEY,robot_model->_q);
    redis_client.createEigenReadCallback(ROBOT_JOINT_VELOCITY_KEY,robot_model->_dq);
    redis_client.createEigenReadCallback(FORCE_SENSOR_FORCE,robot_model->_sensed_force_raw);
    redis_client.createEigenReadCallback(FORCE_SENSOR_MOMENT,robot_model->_sensed_moment_raw);

    redis_client.createEigenWriteCallback(ROBOT_JOINT_TORQUE_KEY,_command_torques);

    // just for testing.
    redis_client.setEigenMatrix(ROBOT_JOINT_TORQUE_KEY,_command_torques);

    Eigen::VectorXd _gravity(n);
    Eigen::VectorXd b(nDof), g(nDof);
    Eigen::MatrixXd M(nDof, nDof);
    Eigen::Vector3d _x;

    Primitives::JointTask* jointTask = new Primitives::JointTask(robot_model);
    jointTask->_target_position = pos;

    // force sensor setup.
    double mass = 0.1;
    Eigen::Vector3d COM = Eigen::Vector3d(0,0,0);
    robot_model->setForceSensorLink("ee_Link",mass,COM);

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    // set the controller key.
    std::string _controller_running = "1";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    while (runloop && timer.WaitForNextLoop())
    {
        redis_client.executeAllReadCallbacks();
        // Do not forget to update the robot model.
        robot_model->updateModel();
        
        jointTask->computeTorque(_command_torques);
        // _command_torques = g;
        // robot_model->position(_x,"ee_Link");
        // std::cout << "_x " << _x.transpose() << std::endl;
        // std::cout << "command torque: \n" << _command_torques << std::endl;
        // std::cout << "q: \n" << robot_model->_q << std::endl;
        robot_model->getForceSensorOutput(_force,_moment);
        std::cout << "force: " << _force.transpose() << std::endl; 
        redis_client.executeAllWriteCallbacks();
        if (jointTask->HasReachedTarget())
            break;
    }
    _controller_running = "0";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    std::cout << "Controller history: " << std::endl;
    timer.printTimerHistory();
    return 0;
}

