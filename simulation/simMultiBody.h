#ifndef _SIM_MULTIBODY_H
#define _SIM_MULTIBODY_H

// Eigen headers.
#include <Eigen/Dense>
#include <Eigen/Core>

#include <simCommonRigidBodyBase.h>
// #include <BulletDynamics/btBulletDynamicsCommon.h>
#include <BulletURDFImporter.h>
#include <BulletInverseDynamics/btBulletCollisionCommon.h>
#include <pugixml.hpp>
#include <sstream>


class simMultiBodyDynamicsWorld: public simCommonRigidBodyBase
{
private:
    /* data */
    int g_constraintSolverType = 1;
    bool g_floatingBase = false;

    btMLCPSolverInterface* m_solverinterface;

    std::unordered_map <std::string, mMultiBody* > _multibody_name_map;
    std::unordered_map <std::string, btRigidBody*> _rigidbody_name_map;

    std::vector<btRigidBody*> _rigidBodyList;
    std::vector<btCollisionShape*> _rigidBodyCollisionShapes;
    std::vector<btMotionState*> _rigidBodyMotionStates;

    std::vector<std::pair<unsigned int, btMultiBodyJointFeedback*>> _force_sensors;

    void convertStringTobtVector3(std::string _vstr, btVector3& _v, std::string _del = " ");
    void convertStringTobtQuaternion(std::string _qstr, btQuaternion& _q, std::string _del = " ");
    
public:
    
    simMultiBodyDynamicsWorld();
    void InitialiseDynamicsWorld();
    void LoadRobotFromURDFFile(std::string _filename, Eigen::Vector3d _base_pose = Eigen::Vector3d(0,0,0),
                                Eigen::Quaterniond _base_rotation = Eigen::Quaterniond(1,0,0,0), bool _fixedBase = true,
                                bool _has_selfcollision = false);
    void LoadFromWorldFile(std::string);
    btConstraintSolver* getSolver();
    btMultiBodyConstraintSolver* getMultiBodySolver();
    void printRobotJointsInfo(RobotObject* _robot);
    void setRobotBasePose(std::string _robotName,double _x, double _y, double _z);
    void setRobotBaseOrientation(std::string _robotName, double _qx, double _qy, double _qz, double _qw);
    void getRobotJointInfo(std::string _robotName);
    btMultiBody* getRobotObject(std::string _robotName);
    btMultiBody* getRobotObject(int index);
    mMultiBody* getMultiBodyObject(int index);
    mMultiBody* getMultiBodyObject(std::string _name) noexcept;
    void getRobotJointPos(int index, Eigen::VectorXd& _q);
    void getRobotJointVel(int index, Eigen::VectorXd& _qd);
    void getRobotJointPos(mMultiBody*,Eigen::VectorXd& _q);
    void getRobotJointVel(mMultiBody*, Eigen::VectorXd& _dq);

    void resetJointPos(mMultiBody*, const Eigen::VectorXd& _q);

    void setRobotJointTorque(mMultiBody*, const Eigen::VectorXd& _t);
    void getRobotJointTorque(mMultiBody*, Eigen::VectorXd& _t);
    void getForceSensorOutput(int _index, Eigen::Vector3d& _force, Eigen::Vector3d& _moment);

    unsigned int addBodyBox(double l, double b, double h, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q);
    unsigned int addBodySphere(double r, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q);
    unsigned int addBodyCylinder(double r, double h, double m, Eigen::Vector3d& _pose, Eigen::Quaterniond& _q);
    unsigned int addBodyConvexHull(std::string _filename,double m, Eigen::Vector3d& _inertia, Eigen::Vector3d _pose,
                         Eigen::Quaterniond _q,Eigen::Vector3d _scale);
    void getBodyPoseAndRotation(unsigned int bodyindex, Eigen::Vector3d& _pos, Eigen::Quaterniond& _q);
    unsigned int attachForceSensorToRobot(std::string _robotName, std::string _linkName);
    unsigned int attachForceSensorToRobot(unsigned int _robotIndex, unsigned int _ind);

    bool getForceMomentFromForceSensor(unsigned int _ind, Eigen::Vector3d&, Eigen::Quaterniond&);

    int getNumRobots() noexcept;
    void stepSimulation(float _ts, float _fixedStep = 0.01);
    ~simMultiBodyDynamicsWorld();

};


struct Nasty
{
    std::string val;
};

#endif