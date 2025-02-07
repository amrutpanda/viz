#pragma once

#include <mRobot.h>
#include <dmObject.h>

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

        // urdf::ModelInterfaceSharedPtr urdf;
        std::string name;
        // std::vector <std::map<std::string, mRobot*>> robots;
        std::map<std::string, mRobot*> robots;
        std::map<std::string, mObject*> objects;
        // std::vector <std::map<std::string, mObject*>> objects;

        Ogre::SceneNode* camNode;
        Ogre::SceneManager* scnMgr;
        OgreBites::CameraMan* mCameraMan;
        OgreBites::ApplicationContextSDL ctx;
        Ogre::Viewport* vp;
        Ogre::RenderWindow* mWin;

        bool* flag = nullptr;
        
        std::string AppName;

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
        void windowResized(Ogre::RenderWindow* rw);
        bool windowClosing(Ogre::RenderWindow* rw);
        void windowClosed(Ogre::RenderWindow* rw);
        void closeGraphics();
        ~mGraphics() {};

        const std::string& getName();
        // urdf::ModelInterfaceSharedPtr getUrdfObject();
        void createRobotObject(std::string _robotName, std::string _robot_filename);
        void createDynamicMeshObject(std::string objName, Eigen::Vector3d pos, Eigen::Quaterniond qrot);
        void creatGraphicalObject(std::string _fileName, std::string objName,Eigen::Vector3d pos, Eigen::Quaterniond qrot,
                                    std::string parent_frame ="");
        
        void updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos);
        void updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos, Eigen::Vector3d base_pose,
                                Eigen::Quaterniond base_rot);
        void setBasePoseAndRotation(std::string _robotName, Eigen::Vector3d _pose,Eigen::Quaterniond _qRotation);

        void createScene();

        mObject* getGraphicalObject(std::string objName);
        mRobot* getRobotObject(std::string _rname);
        // mRobot related functions.


        
    };


    // void createSphere(Ogre::SceneManager* scm, std::string name, Ogre::Vector3 c ,double r, Ogre::MeshPtr* m);
    // void createBox(std::string name, double l, double b, double h, Ogre::Mesh* m);
    // void createCylinder(std::string name, double r, double h, Ogre::Mesh* m);
    // void creatMeshFromFile(std::string filePath,Ogre::String& MeshName);

    // // testing.
    // void say_hello();

} // namespace mviz;