#include <iostream>
#include <redisclient.h>
#include <LoopTimer.h>
#include <dynamics_model.h>
#include <redis_keys.h>
#include <JointTask.h>

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";
std::string control_link = "Link6";
Eigen::Vector3d control_pos_in_link = Eigen::Vector3d(0,0,0);

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
    Eigen::Vector3d _bpose(0,0,0), x_c, x_t;
    Eigen::Quaterniond _brot;
    _brot.setIdentity();

    std::cout << "Starting Jog controller" << std::endl;
    Dynamics::DModel* robot_model = new Dynamics::DModel(robot_file,_bpose,_brot,
                                                          false,false);
    // resize the vectors.
    int n = 0;
    _q.resize(nDof);
    _dq.resize(nDof);
    _command_torques.resize(nDof);

    Eigen::VectorXd pos(6);
    pos.setZero();
    // pos(0) = 0.1;
    pos << -0.1,0.4,-2.0,-0.32,0,0;
    Eigen::Matrix3d orn = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d orn_t = Eigen::Matrix3d::Identity();

    RedisClient redis_client;
    redis_client.connect();
    int r1 = redis_client.createEigenReadCallback(ROBOT_JOINT_POSITION_KEY,robot_model->_q);
    int r2 = redis_client.createEigenReadCallback(ROBOT_JOINT_VELOCITY_KEY,robot_model->_dq);
    int w1 = redis_client.createEigenWriteCallback(ROBOT_JOINT_TORQUE_KEY,_command_torques);
    redis_client.createEigenReadCallback(TARGET_EE_POSE,x_t);
    redis_client.createEigenWriteCallback(CURRENT_EE_POSE,x_c);

    Eigen::VectorXd _gravity(n);
    Eigen::VectorXd b(nDof), g(nDof);
    Eigen::MatrixXd M(nDof, nDof);

    Primitives::JointTask* jointTask = new Primitives::JointTask(robot_model);
    // jointTask->_target_position = pos;
    
    redis_client.getEigenMatrix(ROBOT_JOINT_POSITION_KEY,robot_model->_q);
    redis_client.getEigenMatrix(ROBOT_JOINT_VELOCITY_KEY,robot_model->_dq);
    robot_model->updateKinematics();
    robot_model->position(x_c,control_link);
    robot_model->rotation(orn,control_link);
    // x_c = Eigen::Vector3d(0.3,0.3, 0.4);
    x_t = x_c;
    orn_t = orn;
    

    // send current pose to keyboard reader.
    redis_client.setEigenMatrix(CURRENT_EE_POSE,x_c);
    redis_client.setEigenMatrix(TARGET_EE_POSE,x_t);

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    // set the controller key.
    std::string _controller_running = "1";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    while (runloop && timer.WaitForNextLoop())
    {
        redis_client.executeAllReadCallbacks();
        // Do not forget to update the robot model.
        robot_model->updateModel();
        //compute IK;
        robot_model->computeIK(jointTask->_target_position,control_link,x_t);
        // continue;
        jointTask->computeTorque(_command_torques);
        // _command_torques = g;
    
        // std::cout << "command torque: \n" << _command_torques << std::endl;
        
        // robot_model->positionInWorld(x_c,control_link);
        robot_model->position(x_c,control_link);
        robot_model->rotation(orn,control_link);

        // std::cout << "q: \n" << robot_model->_q << std::endl; 
        // std::cout << "_jposes: \n" << jointTask->_target_position << std::endl;

        // std::cout << "x_t: " << x_t << std::endl;
        // std::cout << "x_c: " << x_c << std::endl;

        std::cout << "rot: " << orn << std::endl;
        std::cout << "==============\n";
        
        redis_client.executeAllWriteCallbacks();
    }
    _controller_running = "0";
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    std::cout << "Controller history: " << std::endl;
    timer.printTimerHistory();
    return 0;
}

