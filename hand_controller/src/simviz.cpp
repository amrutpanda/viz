// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.

#include <iostream>
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

// std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "hand_controller";
std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller.urdf";

std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);


RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q;
// force sensor variables.


int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);

    redis_client.connect();

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);

    // std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.RenderOneFrame();
        if (!runloop)
            std::cout << "runloop flag is false. Exiting..." << std::endl;
    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    // sim_thread.join();
    return 0;
}

void simulation(std::string& _robot_file)
{
    double loop_rate = 200;
        
    LoopTimer timer;
    timer.setLoopFrequency(loop_rate);
    while (runloop & timer.WaitForNextLoop())
    {   
        
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}