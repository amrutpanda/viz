#include <iostream>
// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.
#include <redis_keys.h>
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

// std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";

std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_dq, command_torques;

double l = 0.3;
double b = 0.3;
double h = 0.02;
Eigen::Vector3d boxpos;
Eigen::Quaterniond boxrot;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    _q.resize(nDof);
    _q.setZero();

    _dq.resize(nDof);
    _dq.setZero();

    command_torques.resize(nDof);
    command_torques.setZero();

    redis_client.connect();

    boxpos << -0.3,-0.3,0.25;
    boxrot = Eigen::Quaterniond(1,0,0,0);

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);
    // viz.createBox("box1",l,b,h);
    // viz.setObjectPoseAndRotation("box1",boxpos,boxrot);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q);
        // viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
        viz.RenderOneFrame();
        if (!runloop)
            std::cout << "runloop flag is false" << std::endl;
    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    // sim_thread.join();
    return 0;
}

void simulation(std::string& _robot_file)
{
    int rind = redis_client.createEigenWriteCallback(ROBOT_JOINT_POSITION_KEY,_q);
    int vind = redis_client.createEigenWriteCallback(ROBOT_JOINT_VELOCITY_KEY,_dq);
    int cind = redis_client.createStringReadCallback(CONTROLLER_RUNNING_KEY,_controller_running);

    int tind = redis_client.createEigenReadCallback(ROBOT_JOINT_TORQUE_KEY, command_torques);

    std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
    sim->LoadRobotFromURDFFile(_robot_file);
    sim->setGravity(0, 0, -9.81);
    // int boxid = sim->addBodyBox(l,b,h,0,boxpos,boxrot); // if mass = 0, the object will be static.
    
    RobotObject* robot = sim->getMultiBodyObject(robot_name);
    sim->printRobotJointsInfo(robot);

    Eigen::VectorXd pos(6);
    pos.setZero();
    pos(0) = 0.5;

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    std::cout << "DOF: "  << robot->_jointNameIndexList.size() << std::endl;
    while (runloop & timer.WaitForNextLoop())
    {   
        redis_client.executeAllReadCallbacks();
        sim->getRobotJointPos(robot,_q);
        sim->getRobotJointVel(robot,_dq);
        
        if (_controller_running == "1")
        {   
            // redis_client.executeAllReadCallbacks();
            // std::cout << "command torque: " << command_torques << std::endl;
            // add some damping to the robot joints.
            command_torques = command_torques - 0.1*_dq;
            sim->setRobotJointTorque(robot,command_torques);
            sim->stepSimulation(0.001);

            // sim->getBodyPoseAndRotation(boxid,boxpos,boxrot);
        } 
        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}