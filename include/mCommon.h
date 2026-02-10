#pragma once

#include <Ogre.h>
#include <OgreMesh.h>
#include <OgreCodec.h>
#include <OgreApplicationContext.h>
#include <OgreInput.h>
#include <OgreRTShaderSystem.h>
#include <OgreCameraMan.h>
#include <OgreWindowEventUtilities.h>

#include <iostream>
#include <filesystem>

// Eigen headers
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <urdf_parser/urdf_parser.h>

// C headers
#include <signal.h>
#include <unistd.h>

// #include <Logger.h>

namespace mviz
{
    void createSphere(Ogre::SceneManager* scm, std::string name, Ogre::Vector3 c ,double r, Ogre::MeshPtr* m);
    void createBox(std::string name, double l, double b, double h, Ogre::Mesh* m);
    void createCylinder(std::string name, double r, double h, Ogre::Mesh* m);
    void createMeshFromFile(std::string filePath,Ogre::String& MeshName);
    void createAxisMesh(Ogre::SceneManager* _scnMgr, std::string& _name);

    void convertEigenVecToOgre(Eigen::Vector3d& _eV, Ogre::Vector3& _ogV);
    void convertEigenQuatToOgre(Eigen::Quaterniond &eQuat, Ogre::Quaternion& _ogQuat);

    void convertOgreVecToEigen(Ogre::Vector3& _ogV, Eigen::Vector3d& _eV);
    void convertOgreQuatToEigen(Ogre::Quaternion& _ogQuat, Eigen::Quaterniond& _eQuat);



    // testing.
    void say_hello();
} // namespace mviz
