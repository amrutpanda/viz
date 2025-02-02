
#include "mGraphics.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>

// #include <Ogre.h>
// #include <urdf_parser/urdf_parser.h>

namespace mviz
{
    void mGraphics::readFile(std::string FilePath)
    {
        urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(FilePath);
        
        urdf::LinkConstSharedPtr root = urdf->getRoot();
        
        
    }

    // urdf::ModelInterfaceSharedPtr mGraphics::getUrdfObject()
    // {
    //     return urdf;
    // }


    void mGraphics::urdf_to_ogre_converter(Ogre::SceneManager* scm)
    {
        // to be implemented.
        urdf::LinkSharedPtr link;

    }

    void mGraphics::createRobotObject(std::string _robotName, std::string _robot_filename)
    {
        // Check whether the filename extension is ".urdf".
        std::filesystem::path path(_robot_filename);

        // std::cout << "I am here\n";

        assert(path.extension() == ".urdf");
        // create a new SceneNode from root node and start building robot graphical object from it.
        Ogre::SceneNode* robot_root_node = scnMgr->getRootSceneNode()->createChildSceneNode();
        // create a new robot object.
        mRobot* robot_object = new mRobot(_robotName,_robot_filename,scnMgr,robot_root_node);
        // add robot info to the "robots" object.
        // robots.push_back(std::map<std::string, mRobot*>(_robotName,robot_object));
        robots[_robotName] = robot_object;
        std::cout << robots.at(_robotName)->getName() << std::endl;
        std::cout << "Robot Object created.\n";
        
    }

    void mGraphics::updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos)
    {
        try
        {
            Eigen::VectorXd joint_pos;
            joint_pos = robot_pos; 
            robots.at(_robotName)->updateRobot(joint_pos);
        }
        catch(const std::exception& e)
        {
            std::cout << "updateRobotGraphics: Cannot find a robot with name in the robots list. Name: " 
                      << _robotName << std::endl;
            std::cerr << e.what() << '\n';
            std::quick_exit(-1);
        }
        
    }

    void mGraphics::updateRobotGraphics(std::string _robotName, Eigen::VectorXd robot_pos,
                                        Eigen::Vector3d base_pose, Eigen::Quaterniond base_rot)
    {
        try
        {
            Eigen::VectorXd joint_pos;
            joint_pos = robot_pos; 
            mRobot* robot = robots.at(_robotName);
            robot->setBasePose(base_pose);
            robot->setBaseRotation(base_rot);
            robot->updateRobot(robot_pos);
        }
        catch(const std::exception& e)
        {
            std::cout << "updateRobotGraphics: Cannot find a robot with name in the robots list . Name: " 
                      << _robotName << std::endl;
            std::cerr << e.what() << '\n';
            std::quick_exit(-1);
        }
        
    }

    void mGraphics::setBasePoseAndRotation(std::string _robotName, Eigen::Vector3d _pose, Eigen::Quaterniond _qRotation)
    {
        try
        {
            robots.at(_robotName)->setBasePose(_pose);
            robots.at(_robotName)->setBaseRotation(_qRotation);
        }
        catch(const std::exception& e)
        {
            std::cout << "setBasePoseAndRotation: Cannot find a robot with name in the robots list . Name: " 
            << _robotName << std::endl;
            std::cerr << e.what() << '\n';
            std::quick_exit(-1);
        }
        
    }

    void mGraphics::creatGraphicalObject(std::string _fileName, std::string objName,Ogre::Vector3 pos, Ogre::Quaternion qrot)
    {
        std::string mesh_name;

        std::filesystem::path path(_fileName);
        if ( path.extension() == ".mesh")
        {
            std::cout << "Path extension: " << path.extension() << std::endl;
            mesh_name = path.filename();
        }
        else
        {
            creatMeshFromFile(_fileName,mesh_name);
        }

        mObject* objPtr = new mObject;
        objPtr->objectName = objName;

        Ogre::SceneNode* objSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        objPtr->setSceneNode(objSceneNode);
        
        objPtr->setPosition(pos);
        objPtr->setRotation(qrot);
        objPtr->attachChildMesh(scnMgr,mesh_name, Ogre::Vector3(0,0,0),Ogre::Quaternion(1,0,0,0));   // assuming entity name and mesh name as same at this momemt.
        objects[objName] = objPtr;
        // experimental

        axis* ax = new axis(objPtr);
        
    }

    mObject* mGraphics::getGraphicalObject(std::string objName)
    {
        return objects.at(objName);
    }

    // Graphics related Methods.

    const std::string& mGraphics::getName()
    {
        return name;
    }

    void mGraphics::attachFlagVariable(bool* _flag)
    {
        flag = _flag;
    }

    mGraphics::mGraphics(std::string _name): OgreBites::ApplicationContext(_name)
    {
        AppName = _name;
    }

    void mGraphics::setup()
    {
        // Check if _flag pointer has been assigned.
        if (!flag)
        {
            throw std::runtime_error("Loop flag variable not attached. Call 'attachFlagVariable' function before the render loop.\n");
        }
        
        // OgreBites::ApplicationContext _ctx("Ogreapp");
        // do not forget to call the base.
        // _ctx.setup();
        OgreBites::ApplicationContext::setup();
        addInputListener(this);

        // get a pointer to the already created root
        Ogre::Root* root = getRoot();
        scnMgr = root->createSceneManager();
        
        // register our scene with the RTSS
        Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
        shadergen->addSceneManager(scnMgr);

        // set ambient light.
        scnMgr->setAmbientLight(Ogre::ColourValue(1.0, 1.0, 1.00));

        // set newlight.
        Ogre::Light* light = scnMgr->createLight("MainLight1");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->attachObject(light);

        //! [lightpos]
        lightNode->setPosition(0, 0, 1000);

        // another light node.
        Ogre::Light* light2 = scnMgr->createLight("MainLight2");
        Ogre::SceneNode* lightNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode2->attachObject(light2);

        lightNode2->setPosition(1000, 100, 500);
        // another light node end.

        //! [camera]
        camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

        // create the camera
        Ogre::Camera* cam = scnMgr->createCamera("myCam");
        cam->setNearClipDistance(0.05); // specific to this sample
        cam->setAutoAspectRatio(true);
        // cam->setFOVy(Ogre::Radian(0.08));
        camNode->attachObject(cam);
        camNode->lookAt(Ogre::Vector3(0,0,0),Ogre::Node::TS_WORLD);
        camNode->setPosition(10, 0, 10);

        // create a new Cameraman object and attach it to this object.
        mCameraMan = new OgreBites::CameraMan(camNode);
        mCameraMan->setTopSpeed(1);
        mCameraMan->setStyle(OgreBites::CameraStyle::CS_ORBIT);
        // mCameraMan->setFixedYaw(true);
        addInputListener(mCameraMan);

        // and tell it to render into the main window
        Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
        vp->setBackgroundColour(Ogre::ColourValue(0.0,1.0,1.0));
        // vp->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0));
        //! [camera]

        // get RenderWindow;
        mWin = getRenderWindow();

        // create a new resource group.
        Ogre::ResourceGroupManager::getSingleton().createResourceGroup("UserData");

    }


    bool mGraphics::RenderOneFrame()
    {
        if (*flag)
        {
            return getRoot()->renderOneFrame();
        }
        else
        {
            return false;
        }  
    }

    void mGraphics::closeGraphics()
    {
        closeApp();
    }

    bool mGraphics::keyPressed(const OgreBites::KeyboardEvent &evt)
    {
        if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
        {
            std::cout << "Detected ESC key. Exiting...\n";
            *flag = false;
        }
        return true;
    }

    void mGraphics::windowResized(Ogre::RenderWindow* rw)
    {
        // std::cout << "Window resized\n";
    }

    bool mGraphics::windowClosing(Ogre::RenderWindow* rw)
    {
        std::cout << "Window closing\n";
        return false;
    }

    void mGraphics::windowClosed(Ogre::RenderWindow* rw)
    {
        std::cout << "Window closed\n";
    }

    void mGraphics::addEntity(std::string _FilePath, Ogre::Vector3& pos)
    {
        Ogre::String mesh_name;
        std::filesystem::path path(_FilePath);
        if ( path.extension() == ".mesh")
        {
            std::cout << "Path extension: " << path.extension() << std::endl;
            mesh_name = path.filename();
        }
        else
        {
            creatMeshFromFile(_FilePath,mesh_name);
        }

        // create Entity.
        Ogre::Entity* ogreEntity = scnMgr->createEntity(mesh_name);
        Ogre::SceneNode* ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        ogreNode->attachObject(ogreEntity);
        ogreNode->setPosition(pos);
        
    }
    // ....................... functions not part of Graphics object........................ //
    void creatMeshFromFile(std::string filepath, Ogre::String& MeshName)
    {

        if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("UserData"))
        {
            Ogre::ResourceGroupManager::getSingleton().createResourceGroup("UserData");
        }
        std::filesystem::path path = std::filesystem::canonical(filepath);
        std::cout << path.parent_path() << std::endl;
        std::filesystem::recursive_directory_iterator it(path.parent_path());
        std::filesystem::recursive_directory_iterator end;
        bool res = Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(path.parent_path().generic_string(),
                                                                           "UserData" );
        if (!res)
        {
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().generic_string(),
                                                                        "FileSystem","UserData");
            while (it != end)
            {
                if (it->is_directory())
                {
                    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(it->path().string(),"FileSystem","UserData");
                }   
                ++it;
            }
        }
        else
        {
            std::cout << "Resource location already exists. Skipping ..." << std::endl;
        }
        
        
        // Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().generic_string(),
        //                                                                 "FileSystem","UserData");
        // while (it != end)
        // {
        //     if (it->is_directory())
        //     {
        //         Ogre::ResourceGroupManager::getSingleton().addResourceLocation(it->path().string(),"FileSystem","UserData");
        //     }   
        //     ++it;
        // }

        // std::cout << "Intilising resource group User" << std::endl;
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("UserData");
        // std::cout << "loading resource group User " << std::endl;
        Ogre::ResourceGroupManager::getSingleton().loadResourceGroup("UserData");


        Ogre::String basename, ext, pathname;
        Ogre::StringUtil::splitFullFilename(filepath,basename,ext,pathname);

        std::cout << basename + "." + ext << std::endl;

        // std::cout << "Creating Codec object." << std::endl;
        auto codec = Ogre::Codec::getCodec(ext);

        AssOptions opts;
        opts.source = basename + "." + ext ;

        if (!Ogre::ResourceGroupManager::getSingleton().resourceExists("UserData",basename + "." + ext))
        {
            Ogre::MeshPtr m = Ogre::MeshManager::getSingleton().createManual(basename + "." + ext, "UserData");
            // std::cout << "Decoding the obj file to mesh" << std::endl;
            m->getUserObjectBindings().setUserAny("_AssimpLoaderOptions", opts.options);
            codec->decode(Ogre::Root::openFileStream(opts.source), m.get());
        }
        else
        {
            std::cout << "Resource " << opts.source << " already exists. Skipping ..." << std::endl;
        }
        

        // std::cout << "decoding complete." << std::endl;

        // MeshName = basename + "." + ext;
        MeshName = opts.source;

    }

    void creatAxisMesh(Ogre::SceneManager* _scnMgr, std::string& _name)
    {
        Ogre::ManualObject* pAxis = _scnMgr->createManualObject("axis");
        pAxis->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_STRIP);
        
        
        
    }

    void say_hello()
    {
        std::cout << "Hello. " << std::endl;
    }
} // namespace mviz




