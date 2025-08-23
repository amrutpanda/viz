#include <iostream>
// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.
#include <teleop_redis_keys.h>
#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

// std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "robot1";
std::string robot_name2 = "robot2";
// std::string robot_file = "/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf";
std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";

std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}
void simulation(std::string& _robot_file);
RedisClient redis_client;
unsigned int nDof = 6;
Eigen::VectorXd _q,_dq, command_torques;
Eigen::VectorXd _q2,_dq2,command_torques2;
std::vector<Eigen::VectorXd> _q_;
std::vector<Eigen::VectorXd> _dq_;
std::vector<Eigen::VectorXd> command_torques_;
std::vector<std::string> _controller_running_;

// force sensor variables.
Eigen::Vector3d _force,_moment;
std::vector<Eigen::Vector3d> _force_, _moment_;

double l = 0.3;
double b = 0.3;
double h = 0.1;
Eigen::Vector3d boxpos, boxpos2;
Eigen::Quaterniond boxrot, boxrot2;
// New variable for robots numbers.
int num_robots = 2;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    _q.resize(nDof);
    _q.setZero();

    _dq.resize(nDof);
    _dq.setZero();

    command_torques.resize(nDof);
    command_torques.setZero();

    for (int i = 0; i < num_robots; i++)
    {
        Eigen::VectorXd v;
        v.resize(nDof);
        v.setZero();
        _q_.push_back(v);
        _dq_.push_back(v);
        command_torques_.push_back(v);
        v.resize(3);
        v.setZero();
        _force_.push_back(v);
        _moment_.push_back(v);
        // set controller running key values;
        _controller_running_.push_back("0");
    }
    
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
    // create second robot
    // viz.createRobotObject(robot_name2,robot_file,false,Eigen::Vector3d(0.4 ,1.4 ,0));

    viz.createBox("box1",l,b,h);
    viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
    viz.setObjectColor("box1",0.4,0.5,0.7);

    // viz.createBox("box2",0.82,0.82,0.02);
    // viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);
    // viz.setObjectColor("box2",0.0,0.1,1.0);

    // viz.createGraphicalObject("/home/merai/Downloads/DOBOT - J6 removal_v1.2.dae","sp1",boxpos,boxrot);

    // viz.createSphere("sp1",0.05);
    // viz.setObjectPoseAndRotation("sp1",boxpos2,boxrot2);

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q_[0]);
        // viz.updateRobotGraphics(robot_name2,_q_[1]);
        // boxpos = boxpos + Eigen::Vector3d(0.01,0,0);
        // std::cout << boxpos.transpose() << std::endl;
        // viz.setBasePoseAndRotation(robot_name,boxpos,boxrot);

        // viz.updateRobotGraphics(robot_name,_q,boxpos,boxrot);
        viz.setObjectPoseAndRotation("box1",boxpos,boxrot);
        // viz.setObjectPoseAndRotation("box2",boxpos2,boxrot2);

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
    double loop_rate = 1000;
    
    for (int i = 1; i <= num_robots; i++)
    {
        redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_JOINT_POSITION_KEY,i),_q_[i-1]);
        redis_client.createEigenWriteCallback(createRobotRedisKey(ROBOT_JOINT_VELOCITY_KEY,i),_dq_[i-1]);
        redis_client.createEigenWriteCallback(createRobotRedisKey(FORCE_SENSOR_FORCE,i),_force_[i-1]);
        redis_client.createEigenWriteCallback(createRobotRedisKey(FORCE_SENSOR_MOMENT,i),_moment_[i-1]);

        redis_client.createStringReadCallback(createRobotRedisKey(CONTROLLER_RUNNING_KEY,i),_controller_running_[i-1]);
        redis_client.createEigenReadCallback(createRobotRedisKey(ROBOT_JOINT_TORQUE_KEY,i), command_torques_[i-1]);
        // set default value.
        redis_client.set(createRobotRedisKey(CONTROLLER_RUNNING_KEY,i),_controller_running_[i-1]);
        redis_client.setEigenMatrix(createRobotRedisKey(ROBOT_JOINT_TORQUE_KEY,i),command_torques_[i-1]);
    }
    

    std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
    // Loading the first robot.
    sim->LoadRobotFromURDFFile(_robot_file,Eigen::Vector3d(0,0,0),Eigen::Quaterniond(0,0,0,1),true,false,"robot1");
    // sim->LoadRobotFromURDFFile(robot_file);
    // Loading the second robot.
    // sim->LoadRobotFromURDFFile(_robot_file,Eigen::Vector3d(0.4, 1.4, 0),Eigen::Quaterniond(0,0,0,1),true,false,"robot2");
    sim->setGravity(0, 0, -9.81);
    int boxid = sim->addBodyBox(l,b,h,0,boxpos,boxrot); // if mass = 0, the object will be static.
    // int spid = sim->addBodySphere(0.05,0.1,boxpos2,boxrot2);
    int bid = sim->addBodyBox(0.82, 0.82, 0.02,1.0,boxpos2,boxrot2);

    RobotObject* robot = sim->getMultiBodyObject("robot1");
    // RobotObject* robot2 = sim->getMultiBodyObject("robot2");
    // RobotObject* robot = sim->getMultiBodyObject("cr5_robot");
    sim->printRobotJointsInfo(robot);
    Eigen::Vector<double,6> _q_init;
    _q_init << 1.6,0.3,1.6,0,-1.5,0;
    sim->resetJointPos(robot,_q_init);
    // robot->updateTransforms();
    
    
    // sim->resetJointPos(robot2,_q_init);
    // robot2->updateTransforms();

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
        // sim->getRobotJointPos(robot,_q);
        // sim->getRobotJointVel(robot,_dq);
        // std::cout << "_q: " << _q_[0].transpose() << std::endl;

        sim->getRobotJointPos(robot,_q_[0]);
        sim->getRobotJointVel(robot,_dq_[0]);

        // sim->getRobotJointPos(robot2,_q_[1]);
        // sim->getRobotJointVel(robot2,_dq_[1]);
        
        if (_controller_running_[0] == "1")
        // if (_controller_running_[0] == "1" && _controller_running_[1] == "1")
        {   
            // add some damping to the robot joints.
            sim->setRobotJointTorque(robot,command_torques_[0] - 0.1*_dq_[0]);

            // sim->setRobotJointTorque(robot2,command_torques_[1] - 0.1*_dq_[1]);

            sim->stepSimulation(0.001);
            // robot->updateTransforms();
            // std::cout << robot->_multibody->getLinkCollider(8)->getWorldTransform().getOrigin().x() << std::endl;

            sim->getBodyPoseAndRotation(boxid,boxpos,boxrot);
            sim->getBodyPoseAndRotation(bid,boxpos2,boxrot2);
            sim->getForceSensorOutput(fs_id,_force_[0],_moment_[0]);
            // std::cout << _force_[0].x() << " " << _force_[0].y() << " " << _force_[0].z() << std::endl;
        }
        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "simulation :" << std::endl;
    timer.printTimerHistory();
    std::cout << "sim thread exited" << std::endl;
}