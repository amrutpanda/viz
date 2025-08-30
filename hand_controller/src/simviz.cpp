// TO-DO: include mGraphics before simMultiBody is causing some include errors.
// Need to check on this.
#define RBDL_USE_LOGGING
#include <iostream>
// #include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

#include <dynamics_model.h>

// std::string ROBOT_JOINT_KEY = "robot::q";

std::string robot_name = "hand_controller";
std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller.urdf";
// std::string robot_file = "/home/merai/Downloads/Franka_Research_3_v2/urdf/Franka_Research_3_v2.urdf";
std::string _controller_running = "0";
bool runloop = true;
void sighandler(int signum) {runloop = false;}

inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);  // brings it to (-2π, 2π)
    if (angle < 0)
        angle += 2.0 * M_PI;                   // now in [0, 2π)
    return angle - M_PI;                       // final result in (-π, π]
}
void simulation(std::string& _robot_file);


RedisClient redis_client;
unsigned int nDof = 8;
Eigen::VectorXd _q;
// force sensor variables.


int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);

    redis_client.connect();

    // configure the _q;
    _q.resize(nDof);
    _q.setZero();

    mviz::mVisualizer viz("simviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file);
    viz.getRobotObject(robot_name)->printRobotJointNames();

    std::thread sim_thread(simulation,std::ref(robot_file));

    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        viz.updateRobotGraphics(robot_name,_q);
        viz.RenderOneFrame();
        if (!runloop)
            std::cout << "runloop flag is false. Exiting..." << std::endl;
    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    sim_thread.join();
    return 0;
}

// void simulation(std::string& _robot_file)
// {
//     double loop_rate = 1000;

//     std::unique_ptr<simMultiBodyDynamicsWorld> sim = std::make_unique<simMultiBodyDynamicsWorld>();
//     sim->LoadRobotFromURDFFile(robot_file,Eigen::Vector3d(0,0,0),Eigen::Quaterniond(1,0,0,0),
//                                     true,false,robot_name);
//     sim->setGravity(0,0, -9.81);
//     RobotObject* robot = sim->getMultiBodyObject(robot_name);
//     sim->printRobotJointsInfo(robot);
//     Eigen::VectorXd q,dq;
//     q.resize(nDof);
//     dq.resize(nDof);
//     q.setZero();
//     dq.setZero();
    
//     // add loop closure constraint.
//     sim->addLoopClosureConstraint(robot,4,7,Eigen::Vector3d(0,0,0.3), Eigen::Vector3d(-0.18, -0.04,0));
//     std::cout << "link list:: " << robot->_linkNameIndexList.at(1).second << std::endl;

//     LoopTimer timer;
//     timer.setLoopFrequency(loop_rate);
//     timer.InitializeTimer();
//     while (runloop & timer.WaitForNextLoop())
//     {   
//         sim->getRobotJointPos(robot,q);
//         sim->getRobotJointVel(robot,dq);
//         _q = q;
        
//         sim->setRobotJointTorque(robot,-0.1*dq);
//         sim->stepSimulation(0.001);
//     }
//     std::cout << "simulation :" << std::endl;
//     timer.printTimerHistory();
//     std::cout << "sim thread exited" << std::endl;
// }

// implement rbdl simulation.
void simulation(std::string& robot_file)
{
    Dynamics::DModel sim_model(robot_file,Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
    sim_model.setGravity(Eigen::Vector3d(0,0,-9.81*1));

    // add a constraint.
    // const std::string linkA = "elbow_frame_link1";
    // const std::string linkB = "elbow_frame_link2";

    const std::string linkA = "elbow_link";
    const std::string linkB = "forearm_link";

    Eigen::Affine3d Ta = Eigen::Affine3d::Identity();
    Ta.translation() = Eigen::Vector3d(0,0, 0.3);

    Eigen::Affine3d Tb = Eigen::Affine3d::Identity();
    Tb.translation() = Eigen::Vector3d(-0.18, -0.04, 0);

    const Eigen::Vector3d axis = Eigen::Vector3d(1,1,1);

    sim_model.addLoopConstraint(linkA,linkB,Ta,Tb,Eigen::Vector3i(0,0,0),Eigen::Vector3i(0,0,1));
    sim_model.addLoopConstraint(linkA,linkB,Ta,Tb,Eigen::Vector3i(0,0,0),Eigen::Vector3i(0,1,0));
    sim_model.addLoopConstraint(linkA,linkB,Ta,Tb,Eigen::Vector3i(0,0,0),Eigen::Vector3i(1,0,0));
    sim_model.addLoopConstraint(linkA,linkB,Ta,Tb,Eigen::Vector3i(0,0,1),Eigen::Vector3i(0,0,0));
    sim_model.addLoopConstraint(linkA,linkB,Ta,Tb,Eigen::Vector3i(1,0,0),Eigen::Vector3i(0,0,0));

    // sim_model.getConstraint().Bind(*sim_model.getRBDLModel());
    sim_model.bindConstraint();
    std::cout << "Constraint setup done" << std::endl;
    std::cout << "dof: " << sim_model.dof() << std::endl;

    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    std::cout << "Starting simulation loop" << std::endl;
    Eigen::VectorXd _tau = Eigen::VectorXd::Zero(nDof);

    Eigen::VectorXd constraint_errors;

    sim_model.updateKinematics();

    Eigen::Vector3d posA,posB;
    sim_model.position(posA,"elbow_link",Eigen::Vector3d(0,0,0.3));
    sim_model.position(posB,"forearm_link",Eigen::Vector3d(-0.18,-0.04,0));

    std::cout << "posA: " << posA.transpose() << std::endl;
    std::cout << "posB: " << posB.transpose() << std::endl;
    std::cout << "pos difference: " << (posA - posB).transpose() << std::endl;

    while (runloop && timer.WaitForNextLoop())
    {
        // _tau(1) = -0.5 * sim_model._dq(1); 
        // _tau(nDof - 1) = - 0.1 * sim_model._dq(nDof -1);
        sim_model._tau = - 0.1 *sim_model._dq;
        _tau = sim_model._tau;
        
        // _tau(1) = 10.1;
        _tau(nDof-1) = -0.5;
        _tau(6) = 0.02;
        sim_model._tau = _tau;

        // sim_model.step(0.001);
        // std::cout << "q size: " << sim_model._q.transpose() << std::endl;
        // sim_model._q = sim_model._q.unaryExpr([](double angle) {return normalizeAngle(angle);});
        // _q = sim_model._q;
        // sim_model.updateKinematics();

    }
    timer.printTimerHistory();
}