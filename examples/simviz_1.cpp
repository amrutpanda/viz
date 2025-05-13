#include <iostream>
// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "cr5_robot";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/amrut/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";

bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    _q.resize(nDof);
    _q.setZero();

    redis_client.connect();

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(1500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q);
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
    int rind = redis_client.createEigenWriteCallback(ROBOT_JOINT_KEY,_q);

    std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
    sim->LoadRobotFromURDFFile(_robot_file);
    sim->setGravity(0, 0, -10);
    RobotObject* robot = sim->getMultiBodyObject(robot_name);

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    std::cout << "DOF: "  << robot->_jointNameIndexList.size() << std::endl;
    while (runloop & timer.WaitForNextLoop())
    {
        sim->getRobotJointPos(robot,_q);
        sim->stepSimulation(0.001);
        // std::cout << "sim thread active" << std::endl;
        // usleep(10000); // sleep to observe 
        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}