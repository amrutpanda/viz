#include <iostream>
#include <redisclient.h>
#include <LoopTimer.h>
#include <dynamics_model.h>
#include <teleop_redis_keys.h>
#include <JointTask.h>

double clampVal(double val, double ll, double ul)
{
    if (val < ll)
        return ll;
    else if (val > ul)
        return ul;
    else
        return val;
}

enum State {
    INIT = 0,
    FREE,
    SURGICAL,
    RECENTER
};
std::string state_names[] = {"INIT","FREE","SURGICAL","RECENTER"};

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";
std::string control_link_name = "ee_Link";

bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_dq;
Eigen::VectorXd _command_torques;
// force sensor variables.
Eigen::Vector3d _force, _moment;
// robot number.
int robot_num = 1;

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
    // control pos in link
    Eigen::Vector3d control_pos_in_link = Eigen::Vector3d::Zero();

    // variables to read from haptic controller.
    int haptic_ready = 0;
    redis_client.set(HAPTIC_READY_STATE_KEY, "0");

    int robot_ready = 0;
    redis_client.set(createRobotRedisKey(ROBOT_READY_STATE_KEY,robot_num), std::to_string(robot_ready));

    Eigen::Vector3d _robot_proxy = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _robot_proxy_rot = Eigen::Matrix3d::Identity();
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_TARGET_POS_KEY,robot_num),_robot_proxy);
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_TARGET_ROT_KEY,robot_num),_robot_proxy_rot);

    Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d robot_rot = Eigen::Matrix3d::Identity();

    // read from simulator.
    redis_client.getEigenMatrix(createRobotRedisKey(ROBOT_JOINT_POSITION_KEY,robot_num),robot_model->_q);
    robot_model->updateModel();
    // send default robot pos and rot to redis.
    Eigen::Vector3d _pos_default = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _rot_default = Eigen::Matrix3d::Identity();
    robot_model->position(_pos_default,control_link_name,control_pos_in_link);
    robot_model->rotation(_rot_default,control_link_name);
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_DEFAULT_POS_KEY,robot_num),_pos_default);
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_DEFAULT_ROT_KEY,robot_num),_rot_default);

    Eigen::VectorXd pos(6);
    pos.setZero();
    // pos(0) = 0.1;
    // pos << 0.1,0.4,-2.0,-0.32,0,0;
    pos << 1.6,0.3,1.6,0,-1.5,0;

    RedisClient redis_client;
    redis_client.connect();
    redis_client.createEigenReadCallback(createRobotRedisKey(ROBOT_JOINT_POSITION_KEY,robot_num),robot_model->_q);
    redis_client.createEigenReadCallback(createRobotRedisKey(ROBOT_JOINT_VELOCITY_KEY,robot_num),robot_model->_dq);
    redis_client.createEigenReadCallback(createRobotRedisKey(FORCE_SENSOR_FORCE,robot_num),_force);
    redis_client.createEigenReadCallback(createRobotRedisKey(FORCE_SENSOR_MOMENT,robot_num),_moment);

    // read from haptic device.
    redis_client.createEigenReadCallback(createRobotRedisKey(ROBOT_TARGET_POS_KEY,robot_num),_robot_proxy);
    redis_client.createEigenReadCallback(createRobotRedisKey(ROBOT_TARGET_ROT_KEY,robot_num),_robot_proxy_rot);

    redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_JOINT_TORQUE_KEY,robot_num),_command_torques);
    // to haptic device.
    redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_SENSED_FORCE_KEY,robot_num),_force);
    redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_SENSED_TORQUE_KEY,robot_num),_moment);

    redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_CURRENT_POS_KEY,robot_num),robot_pos);
    redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_CURRENT_ROT_KEY,robot_num),robot_rot);
    redis_client.createIntWriteCallback(ROBOT_READY_STATE_KEY,robot_ready,1);

    // set command torque keys to zero in the beginning.
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_JOINT_TORQUE_KEY,robot_num),_command_torques);

    Eigen::VectorXd _gravity(n);
    Eigen::VectorXd b(nDof), g(nDof);
    Eigen::MatrixXd M(nDof, nDof);
    Eigen::Vector3d _x;


    int state = FREE;
    int prev_state = state;
    bool _surgical_first_loop = true;

    Eigen::Vector3d _fulcrum_pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _fulcrum_rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _fulcrum_offset = Eigen::Vector3d(0,0,0.05);
        
    Primitives::JointTask* jointTask = new Primitives::JointTask(robot_model);
    jointTask->_target_position = pos;
    // jointTask->_target_position = jointTask->_current_position;
    
    // get ee pose from robot model and write to proxies.
    // robot_model->_q = pos;
    robot_model->updateModel();

    robot_model->position(robot_pos,control_link_name,control_pos_in_link);
    robot_model->rotation(robot_rot,control_link_name);
    _robot_proxy = robot_pos;
    _robot_proxy_rot = robot_rot;
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_TARGET_POS_KEY,robot_num),_robot_proxy);
    redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_TARGET_ROT_KEY,robot_num),_robot_proxy_rot);

    // just for testing.
    Eigen::Vector3d _init_pos;
    _init_pos = robot_pos;

    // force sensor setup.
    double mass = 0.1;
    Eigen::Vector3d COM = Eigen::Vector3d(0,0,0);
    robot_model->setForceSensorLink("ee_Link",mass,COM);

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    // set the controller key.
    std::string _controller_running = "1";
    redis_client.set(createRobotRedisKey(CONTROLLER_RUNNING_KEY,robot_num),_controller_running);
    while (runloop && timer.WaitForNextLoop())
    {
        redis_client.executeAllReadCallbacks();
        // Do not forget to update the robot model.
        robot_model->updateModel();
        // set robot target position.
        // std::cout << "target angles: " << jointTask->_target_position << std::endl;
        // std::cout << "current angles: " << jointTask->_current_position << std::endl;
        if (state == INIT)
        {
            robot_model->position(robot_pos,control_link_name,control_pos_in_link);
            robot_model->rotation(robot_rot,control_link_name);
            jointTask->computeTorque(_command_torques);
            if (jointTask->HasReachedTarget())
            {
                jointTask->reInitializeTask();
                break;
            }

        }

        else if (state = FREE)
        {
            robot_model->computeIK(jointTask->_target_position,control_link_name,_robot_proxy,_robot_proxy_rot);
            // std::cout << "robot proxy: " << _robot_proxy << std::endl;
            // std::cout << "_command_torques: " << _command_torques << std::endl;
            // std::cout << "robot joint angles: " << robot_model->_q << std::endl;
            // _command_torques = g;
            robot_model->updateModel();
            robot_model->position(robot_pos,control_link_name,control_pos_in_link);
            robot_model->rotation(robot_rot,control_link_name);
            // robot_model->gravityVector(_command_torques);
            // std::cout << "robot pos: " << robot_pos << std::endl;
            jointTask->computeTorque(_command_torques);
        
        }

        
        // std::cout << "force: " << _force.transpose() << std::endl; 
        redis_client.executeAllWriteCallbacks();
        // if (jointTask->HasReachedTarget())
        //     break;
    }
    _controller_running = "0";
    redis_client.set(createRobotRedisKey(CONTROLLER_RUNNING_KEY,robot_num),_controller_running);
    std::cout << "Controller history: " << std::endl;
    timer.printTimerHistory();
    return 0;
}

