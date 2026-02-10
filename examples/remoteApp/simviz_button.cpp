#include <iostream>
// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.
#include <teleop_redis_keys.h>
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

// std::string ROBOT_JOINT_KEY = "robot::q";

void showNavigationStyle();

struct Pose
{
    Eigen::Vector3d pos;
    Eigen::Vector3d rot;
};

Pose rpose;

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
// force sensor variables.
Eigen::Vector3d _force,_moment;

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

    // boxpos << 0.3,-0.3,0.65;
    boxpos << 0.1,-0.5,0.08;
    boxrot = Eigen::Quaterniond(1,0,0,0);

    // boxpos2 = 1*boxpos + Eigen::Vector3d(-0.4,0.4,3.6);
    boxpos2 = 1*boxpos + Eigen::Vector3d(-0.4,0.4,3.6);

    boxrot2 = boxrot;

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);
    viz.getRobotObject(robot_name)->setRobotAxisVisible(false);

    viz.createBox("box1",l,b,h);
    viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
    viz.setObjectColor("box1",0.4,0.5,0.7);

    // viz.createGraphicalObject("/home/merai/Downloads/Surgical_Room_3D_models/charite_university_hospital_-_operating_room.glb",
    //                             "hosptital", Eigen::Vector3d(0,0,-1.0), Eigen::Quaterniond(1,1.0,0,0),Eigen::Vector3d(1.0,1.0,1.0));
    viz.moveCameraTo(Eigen::Vector3d(0,0,3));
    viz.moveCameraTo(Eigen::Vector3d(0,0,10));

    viz.loadFont();

    // viz.createBox("box2",0.82,0.82,0.02);
    // viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);
    // viz.setObjectColor("box2",0.0,0.1,1.0);

    // viz.createGraphicalObject("/home/merai/Downloads/DOBOT - J6 removal_v1.2.dae","sp1",boxpos,boxrot);

    // viz.createSphere("sp1",0.05);
    // viz.setObjectPoseAndRotation("sp1",boxpos2,boxrot2);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q);
        // boxpos = boxpos + Eigen::Vector3d(0.01,0,0);
        // std::cout << boxpos.transpose() << std::endl;
        // viz.setBasePoseAndRotation(robot_name,boxpos,boxrot);

        // viz.updateRobotGraphics(robot_name,_q,boxpos,boxrot);
        viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
        // viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);

        // showNavigationStyle();

        // viz.RenderOneFrame();
        viz.ImguiRenderOneFrame();
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
    double loop_rate = 1000;
    redis_client.createEigenWriteCallback(ROBOT_JOINT_POSITION_KEY,_q);
    redis_client.createEigenWriteCallback(ROBOT_JOINT_VELOCITY_KEY,_dq);
    redis_client.createEigenWriteCallback(FORCE_SENSOR_FORCE,_force);
    redis_client.createEigenWriteCallback(FORCE_SENSOR_MOMENT,_moment);

    redis_client.createStringReadCallback(CONTROLLER_RUNNING_KEY,_controller_running);
    redis_client.createEigenReadCallback(ROBOT_JOINT_TORQUE_KEY, command_torques);
    
    redis_client.set(CONTROLLER_RUNNING_KEY,_controller_running);
    redis_client.setEigenMatrix(ROBOT_JOINT_TORQUE_KEY,command_torques);

    std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
    sim->LoadRobotFromURDFFile(_robot_file);
    sim->setGravity(0, 0, -9.81);

    int boxid = sim->addBodyBox(l,b,h,0,boxpos,boxrot); // if mass = 0, the object will be static.
    // int spid = sim->addBodySphere(0.05,0.1,boxpos2,boxrot2);
    // int bid = sim->addBodyBox(0.82, 0.82, 0.02,1.0,boxpos2,boxrot2);

    RobotObject* robot = sim->getMultiBodyObject(robot_name);
    sim->printRobotJointsInfo(robot);
    Eigen::Vector<double,6> _q_init;
    _q_init << 1.6,0.3,1.6,0,-1.5,0;
    sim->resetJointPos(robot,_q_init);
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
    std::cout << "Feedback objects: " << robot->_jointFeedbackIndexList.size() << std::endl;
    // force sensor attachment.
    int fs_id = sim->attachForceSensorToRobot(robot,8,0.1);
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
            // std::cout << robot->_multibody->getLinkCollider(8)->getWorldTransform().getOrigin().x() << std::endl;

            sim->getBodyPoseAndRotation(boxid,boxpos,boxrot);
            // sim->getBodyPoseAndRotation(bid,boxpos2,boxrot2);
            sim->getForceSensorOutput(fs_id,_force,_moment);
            // std::cout << "Force: " << _force.transpose() << std::endl;
            // std::cout << "Moment: " << _moment.transpose() << std::endl;
            // std::cout << "simulation running\n";
        }
        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}


void showNavigationStyle()
{
    ImGuiIO& io = ImGui::GetIO();

    // Lock window to left side
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320, io.DisplaySize.y), ImGuiCond_Always);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse;

    if (!ImGui::Begin("Ogre 3D Navigation",nullptr,flags))
    {
        ImGui::End();
        return;
    }


    ImGui::Text("Transform Controls");
    ImGui::Separator();
    ImGui::Button("clickme");
    ImGui::End();
}