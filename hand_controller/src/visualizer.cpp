#include <iostream>
#include <algorithm>

#include <simMultiBody.h>
#include <mGraphics.h>
#include <LoopTimer.h>
#include <redisclient.h>

#include "hc_redis_keys.h"

#include "ParallelRobot.h"
// std::string ROBOT_JOINT_KEY = "robot::q";


bool _simulation = false;
bool _reorder = true;

std::string robot_name = "hand_controller";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge_no_gimbal_v1.1.urdf";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge_v1.1.urdf";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge_no_gimbal.urdf";
// std::string robot_file = "/home/merai/Files/C++/viz/hand_controller/urdf/hand_controller_elbow_hinge.urdf";
// std::string robot_file = "/home/merai/Downloads/000-HAND-CONTROLER-URDF_v1.1/urdf/000-HAND-CONTROLER-URDF_v1.1.urdf";
std::string robot_file = "/home/merai/Downloads/005-HAND-CONTROLER-URDF/urdf/005-HAND-CONTROLER-URDF_v1.0.urdf";


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
unsigned int nDof = 9;
Eigen::VectorXd _q,_q_prev, _delq, _qr,_qr_reordered;
// force sensor variables.


int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    // configure the _q;
    _q.resize(nDof);
    _q.setZero();

    _q_prev.resize(nDof);
    _q_prev.setZero();

    _delq.resize(nDof);
    _delq.setZero();

    // resize the reduced coordinate.
    _qr.resize(nDof - 2);
    _qr.setZero();

    _qr_reordered.resize(nDof - 2);
    _qr_reordered.setZero();

    Eigen::Vector3d pose;
    pose.setZero();

    Eigen::Quaterniond quat;
    quat.setIdentity();
    quat = Eigen::Quaterniond(1,0,0,0);

    // connect to redis server.
    redis_client.connect();

    mviz::mVisualizer viz("viz_hc");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    viz.createRobotObject(robot_name,robot_file,false,pose,quat);
    viz.getRobotObject(robot_name)->printRobotJointNames();
    
    LoopTimer timer;
    timer.setLoopFrequency(500);
    timer.InitializeTimer();

    while (runloop & timer.WaitForNextLoop())
    {
        redis_client.getEigenMatrix(HC_JOINT_POSITION_KEY,_qr);
        if (_reorder)
            std::rotate(_qr.data()+2, _qr.data()+6, _qr.data()+7);
        viz.updateRobotGraphics(robot_name,_q);
        viz.RenderOneFrame();
        // if (!runloop)
        //     std::cout << "runloop flag is false. Exiting..." << std::endl;
        
        if (!_simulation && (nDof == 5))
        {
            _q(0) = _qr(0);
            _q(1) = _qr(1);
            _q(nDof -2 ) = _qr(2);

            _delq = _q - _q_prev;

            _q(2) =  _q(2) + (_delq(1) - _delq(nDof -2))*(-1);
            _q(nDof - 1) = _q(nDof - 1) + (_delq(nDof -2) - _delq(1))*(-1);

            _q_prev = _q;
        }
        else if (!_simulation && (nDof > 5))
        {
            _q(0) = _qr(0);
            _q(1) = _qr(1);
            _q(nDof -2 ) = _qr(2);

            _q(3) = _qr(3);
            _q(4) = _qr(4);
            _q(5) = _qr(5);
            _q(6) = _qr(6);

            _delq = _q - _q_prev;

            _q(2) =  _q(2) + (_delq(1) - _delq(nDof -2))*(-1);
            _q(nDof - 1) = _q(nDof - 1) + (_delq(nDof -2) - _delq(1))*(-1);

            // _q(2) =  _q(2) + (_delq(nDof -2) - _delq(1));
            // _q(nDof - 1) = _q(nDof - 1) + (_delq(1) - _delq(nDof -2));

            _q_prev = _q;   
        }

        // double _dq1 = 0.00002;
        // double _dq2 = 0.00002;
        // _q(1) = _q(1) + _dq1;
        // _q(6) = _q(6) + _dq2;
        // _q(7) = _q(7) +  (_dq1 - _dq2);
        // _q(2) = _q(2) + (_dq2 - _dq1);

        // _q(nDof -1) = (M_PI + (M_PI_2 + _q(1)) - ( M_PI + _q(nDof - 2)) - M_PI_2);
        // _q(2) = (M_PI + _q(nDof - 2)) -( M_PI_2 + _q(1));

    }
    std::cout << "Visualization" << std::endl;
    timer.printTimerHistory();
    return 0;
}
