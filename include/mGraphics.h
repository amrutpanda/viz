#pragma once

#include <mRobot.h>

// #include <filesystem>
// #include <urdf_parser/urdf_parser.h>
// #include <mRobot.h>
// #include <OgreCodec.h>
// #include <OgreApplicationContext.h>
// #include <OgreInput.h>
// #include <OgreRTShaderSystem.h>
// #include <OgreCameraMan.h>

namespace mviz{

    struct AssOptions
    {
        Ogre::String source;
        Ogre::String dest;
        Ogre::String logFile;

        Ogre::BinaryOptionList options;

        AssOptions() { logFile = "OgreAssimp.log"; };
    };

    class mGraphics 
            :   public OgreBites::ApplicationContext, 
                public OgreBites::InputListener
    {
    private:

        std::string resource_group_name = "User";

        urdf::ModelInterfaceSharedPtr urdf;
        std::string name;
        std::vector <std::map<std::string, mRobot>> robots;
        std::vector <std::map<std::string, mObject>> objects;

        Ogre::SceneNode* camNode;
        Ogre::SceneManager* scnMgr;
        OgreBites::CameraMan* mCameraMan;
        OgreBites::ApplicationContextSDL ctx;
        Ogre::Viewport* vp;

        bool* flag;

    public:
    // mGraphics(/* args */) {};
        mGraphics(std::string _name);
        void readFile( std::string FilePath);
        void urdf_to_ogre_converter(Ogre::SceneManager* scm);
        void addEntity(std::string filePath, Ogre::Vector3& position); // only position at this moment, orientation later.
        void attachFlagVariable(bool* _flag);
        void setup();
        bool RenderOneFrame();
        bool keyPressed(const OgreBites::KeyboardEvent &evt);
        void closeGraphics();
        ~mGraphics() {};

        const std::string& getName();
        urdf::ModelInterfaceSharedPtr getUrdfObject();

        void createScene();


        
    };


    // void createSphere(Ogre::SceneManager* scm, std::string name, Ogre::Vector3 c ,double r, Ogre::MeshPtr* m);
    // void createBox(std::string name, double l, double b, double h, Ogre::Mesh* m);
    // void createCylinder(std::string name, double r, double h, Ogre::Mesh* m);
    // void creatMeshFromFile(std::string filePath,Ogre::String& MeshName);

    // // testing.
    // void say_hello();

} // namespace mviz;