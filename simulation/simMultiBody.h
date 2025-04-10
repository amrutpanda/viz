#ifndef _SIM_MULTIBODY_H
#define _SIM_MULTIBODY_H

// Eigen headers.
#include <Eigen/Dense>
#include <Eigen/Core>

#include <simCommonRigidBodyBase.h>
// #include <BulletDynamics/btBulletDynamicsCommon.h>
#include <BulletURDFImporter.h>


class simMultiBodyDynamicsWorld: public simCommonRigidBodyBase
{
private:
    /* data */
    int g_constraintSolverType = 1;
    bool g_floatingBase = false;

    btMLCPSolverInterface* m_solverinterface;

    std::unordered_map <std::string, mMultiBody* > _multibody_name_map;
    std::unordered_map <std::string, btRigidBody*> _rigidbody_name_map;
    
public:
    
    simMultiBodyDynamicsWorld();
    void InitialiseDynamicsWorld();
    void LoadRobotFromURDFFile(std::string _filename);
    void setRobotBasePose(std::string _robotName,double _x, double _y, double _z);
    void setRobotBaseOrientation(std::string _robotName, double _qx, double _qy, double _qz, double _qw);
    void getRobotJointInfo(std::string _robotName);
    btMultiBody* getRobotObject(std::string _robotName);
    btMultiBody* getRobotObject(int index);
    mMultiBody* getMultiBodyObject(int index);
    void getRobotJointPos(int index, Eigen::VectorXd& _q);
    void getRobotJointVel(int index, Eigen::VectorXd& _qd);
    void getRobotJointsPos(mMultiBody*,Eigen::VectorXd& _q);
    void getRobotJointsVel(mMultiBody*, Eigen::VectorXd& _dq);
    void setRobotJointTorque(mMultiBody*, Eigen::VectorXd& _t);
    void getRobotJointTorque(mMultiBody*, Eigen::VectorXd& _t);
    
    int getNumRobots();
    void step(int _ts);
    ~simMultiBodyDynamicsWorld();

    

};


struct Nasty
{
    std::string val;
};

#endif