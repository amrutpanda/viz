#pragma once

#include <Ogre.h>
#include <OgreMesh.h>
#include <OgreCodec.h>
#include <OgreApplicationContext.h>
#include <OgreInput.h>
#include <OgreRTShaderSystem.h>
#include <OgreCameraMan.h>


#include <iostream>
#include <filesystem>

namespace mviz
{
    void createSphere(Ogre::SceneManager* scm, std::string name, Ogre::Vector3 c ,double r, Ogre::MeshPtr* m);
    void createBox(std::string name, double l, double b, double h, Ogre::Mesh* m);
    void createCylinder(std::string name, double r, double h, Ogre::Mesh* m);
    void creatMeshFromFile(std::string filePath,Ogre::String& MeshName);

    // testing.
    void say_hello();
} // namespace mviz
