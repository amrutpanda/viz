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
double h = 0.1;
Eigen::Vector3d boxpos, boxpos2;
Eigen::Quaterniond boxrot, boxrot2;

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

    boxpos << 0.3,-0.3,0.65;
    boxrot = Eigen::Quaterniond(1,0,0,0);

    boxpos2 = 1*boxpos + Eigen::Vector3d(-0.4,0.4,3.6);
    boxrot2 = boxrot;

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);

    viz.createBox("box1",l,b,h);
    viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
    viz.setObjectColor("box1",0.4,0.5,0.7);

    viz.createBox("box2",0.82,0.82,0.02);
    viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);
    viz.setObjectColor("box2",0.0,0.1,1.0);

    // viz.createGraphicalObject("/home/merai/Downloads/DOBOT - J6 removal_v1.2.dae","sp1",boxpos,boxrot);

    // viz.createSphere("sp1",0.05);
    // viz.setObjectPoseAndRotation("sp1",boxpos2,boxrot2);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q);
        // boxpos = boxpos + Eigen::Vector3d(0.01,0,0);
        // std::cout << boxpos.transpose() << std::endl;
        // viz.setBasePoseAndRotation(robot_name,boxpos,boxrot);

        // viz.updateRobotGraphics(robot_name,_q,boxpos,boxrot);
        viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
        viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);

        viz.RenderOneFrame();
        if (!runloop)
            std::cout << "runloop flag is false. Exiting..." << std::endl;
    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    sim_thread.join();
    return 0;
}

void simulation(std::string& _robot_file)
{
    double loop_rate = 200;
    int rind = redis_client.createEigenWriteCallback(ROBOT_JOINT_POSITION_KEY,_q);
    int vind = redis_client.createEigenWriteCallback(ROBOT_JOINT_VELOCITY_KEY,_dq);
    int cind = redis_client.createStringReadCallback(CONTROLLER_RUNNING_KEY,_controller_running);

    int tind = redis_client.createEigenReadCallback(ROBOT_JOINT_TORQUE_KEY, command_torques);
    
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    redis_client.setEigenMatrix(ROBOT_JOINT_TORQUE_KEY,command_torques);

    std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
    sim->LoadRobotFromURDFFile(_robot_file);
    sim->setGravity(0, 0, -9.81);

    int boxid = sim->addBodyBox(l,b,h,0,boxpos,boxrot); // if mass = 0, the object will be static.
    // int spid = sim->addBodySphere(0.05,0.1,boxpos2,boxrot2);
    int bid = sim->addBodyBox(0.82, 0.82, 0.02,100.8,boxpos2,boxrot2);

    RobotObject* robot = sim->getMultiBodyObject(robot_name);
    sim->printRobotJointsInfo(robot);
    Eigen::Vector<double,6> _q_init;
    _q_init << 1.6,0.3,1.6,0,-1.5,0;
    // sim->resetJointPos(robot,_q_init);
    robot->updateTransforms();
    // for (auto it : robot->_linkNameIndexList)
    // {
    //     std::cout << "Link_name: " << it.second << " " << "Index: " << it.first << std::endl;
    // }

    Eigen::Vector3d Force, Moment;
    
    Eigen::VectorXd pos(6);
    pos.setZero();
    pos(0) = 0.5;
    pos(1) = 0.5;

    // sim->setRobotBasePose(robot_name,boxpos.x(),boxpos.y(),boxpos.z()); // this causes nan value in joint reading.
    
    LoopTimer timer;
    timer.setLoopFrequency(loop_rate);
    timer.InitializeTimer();
    std::cout << "DOF: "  << robot->_jointNameIndexList.size() << std::endl;
    // force sensor attachment.
    sim->attachForceSensorToRobot(robot,9,0.1);
    while (runloop & timer.WaitForNextLoop())
    {   
        redis_client.executeAllReadCallbacks();
        sim->getRobotJointPos(robot,_q);
        sim->getRobotJointVel(robot,_dq);
        
        if (_controller_running == "1")
        {   
            // add some damping to the robot joints.
            command_torques = command_torques - 0.1*_dq;
            sim->setRobotJointTorque(robot,command_torques);
            sim->stepSimulation(0.001);
            // robot->updateTransforms();

            sim->getBodyPoseAndRotation(boxid,boxpos,boxrot);
            sim->getBodyPoseAndRotation(bid,boxpos2,boxrot2);
            sim->getForceSensorOutput(robot,8,Force,Moment);
            std::cout << Force.transpose() << std::endl;
            // std::cout << "simulation running\n";
        }
        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}