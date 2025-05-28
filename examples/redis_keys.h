#include <iostream>

std::string CONTROLLER_RUNNING_KEY = "robot::controller_key";
std::string ROBOT_JOINT_POSITION_KEY = "robot::q";
std::string ROBOT_JOINT_VELOCITY_KEY = "robot::dq";
std::string ROBOT_JOINT_TORQUE_KEY = "robot::command_torque";

// required for keyboard control.
std::string CURRENT_EE_POSE = "robot::ee::pose::current";
std::string TARGET_EE_POSE = "robot::ee::pose::target";