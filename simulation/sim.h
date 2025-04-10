// #ifndef SIM_H
// #define SIM_H
#pragma once

#include <iostream>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <Eigen/Dense>
#include <Logger.hpp>
// assimp header files
#include <assimp/cimport.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
class Simulation
{
private:
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;

    bool memReleased = false;
    bool first_loop = true;
    double _fixedTimeStep = 0.001;
    double _maxSubsteps = 1;
    bool checkIfrBodyExists(std::string _name);
protected:
    btDiscreteDynamicsWorld* dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
    std::vector<btRigidBody*>_rigidBodies;
    std::vector<btMotionState*> _MotionStates;
    std::vector<std::string> rigidBodyNames;
    Logger* log;
public:
    Simulation(/* args */);
    void run(double TimeStep);
    /**
     * setGravity: x,y,z components of gravity;
    */
    void setGravity(double _gx, double _gy, double _gz);
    /**
     * create a box shaped rigid body
     * return the index of the rigid body stored inside simulation object.
    */
    unsigned int addBodyBox(std::string _name, double l, double b, double h, double m, 
                    Eigen::Vector3d _pose, Eigen::Quaterniond _q = Eigen::Quaterniond(1,0,0,0));
    unsigned int addBodySphere(std::string _name, double r, double m,
                        Eigen::Vector3d _pose, Eigen::Quaterniond _q = Eigen::Quaterniond(1,0,0,0));
    unsigned int addBodyCylinder(std::string _name, double r, double h,double m,
                            Eigen::Vector3d _pose, Eigen::Quaterniond _q = Eigen::Quaterniond(1,0,0,0));
    unsigned int addBodyFromFile(std::string _name, std::string _filename, 
                                    Eigen::Vector3d _pose, Eigen::Quaterniond _q = Eigen::Quaterniond(1,0,0,0),
                                    Eigen::Vector3d _scale = Eigen::Vector3d(1,1,1));
    void stepSimulation(double _timesteps);
    void getBodyPoseAndRotation(int bodyIndex, Eigen::Vector3d& _pos, Eigen::Quaterniond& _q);
    void applyForce(int bodyIndex, Eigen::Vector3d _force);
    void close();
    ~Simulation();
};




// #endif