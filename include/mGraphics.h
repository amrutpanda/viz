#pragma once

#include <mRobot.h>
#include <dmObject.h>
#include <mCamera.h>
#include <OgreTimer.h>
#include <OgreImGuiOverlay.h>
#include <OgreFontManager.h>
#include <OgreImGuiInputListener.h>
#include <OgreOverlayManager.h>
#include <OgreOverlaySystem.h>
#include <OgreStringConverter.h>

// tinyXml2 libraries.
#include <pugixml.hpp>

// SDL Libraries;
#include <SDL.h>
#include <SDL_video.h>
#include <SDL_syswm.h>
// OgreBites SDLInputmapping.
#include "SDLInputMapping.h"


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
        OgreBites::CameraMan* mCameraMan;
        OgreBites::ApplicationContextSDL ctx;
        Ogre::Viewport* vp;
        Ogre::RenderWindow* mWin;
        
        bool* flag = nullptr;
        
        std::string AppName;
        
        mObject* findFrameObjectPtr(std::string& name);
    protected:
        void pollWindowEvents();
        Ogre::SceneManager* scnMgr;
        
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
        void createRobotObject(std::string _robotName, std::string _robot_filename, 
                               Eigen::Vector3d _bpos = Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond _brot = Eigen::Quaterniond::Identity());
        void createDynamicMeshObject(std::string objName, Eigen::Vector3d pos, Eigen::Quaterniond qrot, std::string parent_frame = "");
        void createGraphicalObject(std::string _fileName, std::string objName,Eigen::Vector3d pos, Eigen::Quaterniond qrot,
                                    std::string parent_frame ="");
        
        void updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos);
        void updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos, Eigen::Vector3d base_pose,
                                Eigen::Quaterniond base_rot);
        void setBasePoseAndRotation(std::string _robotName, Eigen::Vector3d _pose,Eigen::Quaterniond _qRotation);
        void setRobotMeshOrientation(std::string& _robotName, double angle, int axis); // angle is in Degree, axis has to be from enum AXIS;

        void setObjectPoseAndRotation(std::string _Name, Eigen::Vector3d _pose,Eigen::Quaterniond _qRotation);
        void createBox(std::string _name, float l, float b, float h, std::string _parent = "");
        void createSphere(std::string _name, float r, std::string parent="");
        void createCylinder(std::string _name, float r, float h, std::string parent="");
        void createScene();
        void createLine();

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


    struct ImguiWindow : Ogre::FrameListener
    {
        bool saveConfig;

        bool frameStarted(const Ogre::FrameEvent& evt) override
        {
            
            Ogre::ImGuiOverlay::NewFrame();
            auto flags = ImGuiWindowFlags_AlwaysAutoResize;
            // std::cout << "I am here" << std::endl;
            ImGui::Begin("Hello",NULL,flags);
            ImGui::Text("I am here");
            ImGui::End();
            // ImGui::Render();
            return true;
        }

    };

    class mVisualizer : public mGraphics
    {
    private:
        std::string gui_name;
        Ogre::ImGuiOverlay* overlay;
        struct ImguiWindow* imgui_window;
        double frame_rate;
        Ogre::Timer timer;
        double prev_time = 0;
        // std::unique_ptr<OgreBites::InputListener> mImGuiListener;
    public:
        mVisualizer(std::string _name);
        ~mVisualizer() {};
        bool ImguiRenderOneFrame();
        void loadFont();

        // bool frameStarted(const Ogre::FrameEvent& evt);
        // void setup();
    };
    

} // namespace mviz;