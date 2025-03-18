
#include "mGraphics.h"
#include "OgreWindowEventUtilities.h"
#include "OgreLogManager.h"
// #include <Ogre.h>
// #include <urdf_parser/urdf_parser.h>


namespace mviz
{
    void mGraphics::readFile(std::string FilePath)
    {
        urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(FilePath);
        
        urdf::LinkConstSharedPtr root = urdf->getRoot();

        // check for correct file extension.

        std::filesystem::path _path = std::filesystem::canonical(FilePath);
        if (_path.extension() != ".world")
            throw std::runtime_error("Loading failed. Please load a file with extension '.world'...");
        // tinyxml2 parsing

        // tinyxml2::XMLDocument doc;
        // doc.LoadFile(_path.c_str());
        
        // doc.Print();
        

        
        
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

    void mGraphics::createRobotObject(std::string _robotName, std::string _robot_filename, 
                                      Eigen::Vector3d _bpos ,Eigen::Quaterniond _brot)
    {
        // Check whether the filename extension is ".urdf".
        std::filesystem::path path(_robot_filename);

        // std::cout << "I am here\n";

        // assign base_pos and orientation.

        assert(path.extension() == ".urdf");
        // create a new SceneNode from root node and start building robot graphical object from it.
        Ogre::SceneNode* robot_root_node = scnMgr->getRootSceneNode()->createChildSceneNode();
        // create a new robot object.
        mRobot* robot_object = new mRobot(_robotName,_robot_filename,scnMgr,robot_root_node, _bpos, _brot);
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

    void mGraphics::creatGraphicalObject(std::string _fileName, std::string objName, Eigen::Vector3d _pos, Eigen::Quaterniond _qrot, std::string parent_frame)
    {
        std::string mesh_name;
        Ogre::Vector3 pos;
        Ogre::Quaternion qrot;

        convertEigenVecToOgre(_pos,pos);
        convertEigenQuatToOgre(_qrot,qrot);

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

        Ogre::SceneNode* objSceneNode;
        mObject* optr = nullptr;
        if (!parent_frame.empty())
            optr = findFrameObjectPtr(parent_frame);
        
        if (optr == nullptr)
            objSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        else
            objSceneNode = optr->getSceneNode()->createChildSceneNode();
        objPtr->setSceneNode(objSceneNode);
        
        objPtr->setPosition(pos);
        objPtr->setRotation(qrot);
        objPtr->attachChildMesh(scnMgr,mesh_name, Ogre::Vector3(0,0,0),Ogre::Quaternion(1,0,0,0));   // assuming entity name and mesh name as same at this momemt.
        objects[objName] = objPtr;
        // experimental

        axis* ax = new axis(objPtr);
        
    }

    void mGraphics::createDynamicMeshObject(std::string objName, Eigen::Vector3d _pos, Eigen::Quaterniond _qrot, std::string parent_frame)
    {
        Ogre::Vector3 pos;
        Ogre::Quaternion qrot;

        convertEigenVecToOgre(_pos, pos);
        convertEigenQuatToOgre(_qrot,qrot);

        dmObject* objPtr = new dmObject();
        objects[objName] = objPtr;

        Ogre::SceneNode* objSceneNode;
        mObject* optr = nullptr;
        if (!parent_frame.empty())
            optr = findFrameObjectPtr(parent_frame);
        
        if (optr == nullptr)
            objSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        else
            objSceneNode = optr->getSceneNode()->createChildSceneNode();
        
        objPtr->setSceneNode(objSceneNode);

        objPtr->setPosition(pos);
        objPtr->setRotation(qrot);

        objPtr->createManualObject("pc");

        // attach Manual object mesh.

    }

    mObject* mGraphics::findFrameObjectPtr(std::string& _fname)
    {
        // first search among robots.
        for (auto it = robots.begin(); it != robots.end(); ++it)
        {
            mRobotLink* rl_ptr = it->second->getRobotLinkFromFrameName(_fname);
            if(rl_ptr != nullptr)
                return dynamic_cast<mObject*>(rl_ptr);
        }
        for (auto it = objects.begin(); it != objects.end(); ++it)
        {
            if (it->first == _fname)
                return it->second;
        }
        return nullptr;
    }

    mObject* mGraphics::getGraphicalObject(std::string objName)
    {
        return objects.at(objName);
    }

    mRobot* mGraphics::getRobotObject(std::string _rname)
    {
        return robots.at(_rname);
    }

    void mGraphics::setRobotMeshOrientation(std::string& _robotName, double angle, int axis)
    {
        mRobot* robot;
        try
        {
            robot = robots.at(_robotName);
        }
        catch(const std::exception& e)
        {
            throw std::runtime_error("Inside setRobotMeshOrientation: Could not find the robot with given name: " + _robotName);
        }
        
        robot->flipDAEMeshes(angle,axis);
    }

    // ********** +++++++++++++ ***************
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
            throw std::runtime_error("Loop flag variable not attached. Call 'attachFlagVariable' function InitApp.\n");
        }
        
        
        Ogre::LogManager* logMgr = Ogre::LogManager::getSingletonPtr();
        logMgr->createLog("Mylog",true, false, false);
        // get a pointer to the already created root
        Ogre::Root* root = getRoot();
        scnMgr = root->createSceneManager();
        //
        root->initialise(false);
        Ogre::NameValuePairList options;
        // options["vsync"] = true;   // to deal with FPS being stuck to refresh rate of monitor.
        OgreBites::NativeWindowPair p = createWindow(AppName,860,640,options);
        locateResources();
        initialiseRTShaderSystem();
        loadResources();
        root->addFrameListener(this);
        //

        // OgreBites::ApplicationContext _ctx("Ogreapp");
        // do not forget to call the base.
        // _ctx.setup();
        // OgreBites::ApplicationContext::setup();

        addInputListener(this);
        
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

        // Ogre setup skydome;
        // scnMgr->setSkyDome(true,"Examples/CloudySky", 5, 8);

        //! [camera]
        camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

        // create the camera
        Ogre::Camera* cam = scnMgr->createCamera("myCam");
        cam->setNearClipDistance(0.05); // specific to this sample
        cam->setAutoAspectRatio(true);
        // cam->setFOVy(Ogre::Radian(0.08));
        camNode->attachObject(cam);
        camNode->lookAt(Ogre::Vector3(0,0,0),Ogre::Node::TS_WORLD);
        camNode->setPosition(0, 0, 10);

        // create a new Cameraman object and attach it to this object.
        mCameraMan = new OgreBites::CameraMan(camNode);
        mCameraMan->setTopSpeed(1);
        mCameraMan->setStyle(OgreBites::CameraStyle::CS_ORBIT);
        addInputListener(mCameraMan);

        // adding custom camera
        // mCamera* Camera = new mCamera(camNode);
        // addInputListener(Camera);
        

        // and tell it to render into the main window
        Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
        vp->setBackgroundColour(Ogre::ColourValue(0.0,1.0,1.0));
        // vp->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0));
        //! [camera]

        // get RenderWindow;
        // mWin = getRenderWindow();
        
                
        
        // create a new resource group.
        Ogre::ResourceGroupManager::getSingleton().createResourceGroup("UserData");
        // adding the resource folder to Ogre "FileSystem".
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/asp/Files/cpp/projects/viz/resources",
                                                                        "FileSystem","UserData",true);
        // now initialise the resourcegroup.
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("UserData");

    }


    bool mGraphics::RenderOneFrame()
    {
        if (*flag)
        {
            pollWindowEvents();
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

    void mGraphics::pollWindowEvents()
    {
        if (mWindows.empty())
            return;
        
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
            {
                for (auto & window : mWindows)
                {
                    if(event.window.windowID != SDL_GetWindowID(window.native))
                        continue;
                    Ogre::RenderWindow* win = window.render;
                    mRoot->queueEndRendering();
                    windowClosed(win);
                }
                break;
            }
            case SDL_WINDOWEVENT:
            {
                if(event.window.event != SDL_WINDOWEVENT_RESIZED)
                    continue;
                for(auto & window : mWindows)
                {
                    if(event.window.windowID != SDL_GetWindowID(window.native))
                        continue;
    
                    Ogre::RenderWindow* win = window.render;
                    win->resize(event.window.data1, event.window.data2);
                    windowResized(win);
                }
                break;
            }
            
            default:
                _fireInputEvent(convert(event), event.window.windowID);
                break;
            }
        }
        
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
        std::cout << "Application closed\n";
        destroyWindow(AppName);
        *flag = false;
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

        // if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("UserData"))
        // {
        //     Ogre::ResourceGroupManager::getSingleton().createResourceGroup("UserData");
        // }

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
        

        // std::cout << "Intilising resource group User" << std::endl;
        // Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("UserData");
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

    void convertEigenVecToOgre(Eigen::Vector3d& _eV, Ogre::Vector3& _ogV)
    {
        _ogV.x = _eV.x();
        _ogV.y = _eV.y();
        _ogV.z = _eV.z();
    }

    void convertEigenQuatToOgre(Eigen::Quaterniond &_eQuat, Ogre::Quaternion& _ogQuat)
    {
        _ogQuat.w = _eQuat.w();
        _ogQuat.x = _eQuat.x();
        _ogQuat.y = _eQuat.y();
        _ogQuat.z = _eQuat.z();
    }



    void say_hello()
    {
        std::cout << "Hello. " << std::endl;
    }
} // namespace mviz

// mGui implementation
namespace mviz
{
    mVisualizer::mVisualizer(std::string _name) : mGraphics(_name)
    {
        std::cout << "Created visualizer instance." << std::endl;
        gui_name = _name;
    }

    void mVisualizer::loadFont()
    {
        scnMgr->addRenderQueueListener(mOverlaySystem);
        float vpScale = getDisplayDPI()/96;
        Ogre::OverlayManager::getSingleton().setPixelRatio(vpScale);
        overlay = initialiseImGui();
        ImGui::GetIO().FontGlobalScale = std::round(vpScale);
        ImGui::GetIO().FontGlobalScale = 0.27;
        // ImGui::ShowDemoWindow();
        
        overlay->addFont("Roboto-Medium", "UserData");
        overlay->show();
        addInputListener(getImGuiInputListener());
        std::cout << "Adding font" << std::endl;

    }
    
    bool mVisualizer::ImguiRenderOneFrame()
    {
        auto flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove
                        | ImGuiWindowFlags_NoTitleBar;
        overlay->NewFrame();
        // std::cout << "I am here" << std::endl;
        ImGui::SetNextWindowSize(ImVec2(400,100));
        ImGui::SetNextWindowBgAlpha(0.1);
        ImGui::Begin("Frame Rate ",NULL,flags);
        ImGui::Text("Frame Rate: %f",frame_rate);
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Options")) {
                if (ImGui::MenuItem("Show Axis")) { 
                    std::cout << "Clicked Show Axis" << std::endl;
                }
                if (ImGui::MenuItem("Show Joint value")) { 
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
        ImGui::End();
        frame_rate = 1000/(timer.getMilliseconds() - prev_time);
        // std::cout << "time: " << frame_rate << std::endl;
        prev_time = timer.getMilliseconds();
        return RenderOneFrame();
    }

    // bool mVisualizer::frameStarted(const Ogre::FrameEvent& evt)
    // {
        
    // }
} // namespace mviz





