
#include "mGraphics.h"
#include "OgreWindowEventUtilities.h"
#include "OgreLogManager.h"
// #include <Ogre.h>
// #include <urdf_parser/urdf_parser.h>


namespace mviz
{
    void mGraphics::readFile(std::string FilePath)
    {
        // urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(FilePath);
        
        // urdf::LinkConstSharedPtr root = urdf->getRoot();

        // check for correct file extension.

        std::filesystem::path _path = std::filesystem::canonical(FilePath);
        if (_path.extension() != ".world")
            throw std::runtime_error("Loading failed. Please load a file with extension '.world'...");
        // pugixml parsing
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(FilePath.c_str());
        if (!result)
            throw std::runtime_error("Error while reading the WORLD file.\n");
        pugi::xml_node root_node = doc.root().child("world");
        // std::cout << "pugi node name: " << root_node.child("world").name() << std::endl;
        
        
        for (const pugi::xml_node _node : root_node)
        {   
            pugi::xml_node tNode;
            std::string name, type, filename, tval;
            Ogre::Vector3 _xyz, _rpyV;
            Ogre::Quaternion _rpy;
            Ogre::Vector3 _scale;
            Eigen::Vector3d xyz;
            Eigen::Quaterniond rpy;
            Eigen::Vector3d scale;
            double l,b,h,r;
            // std::cout << _node.name() << std::endl;
            std::string _node_name = _node.name();
            if (_node_name == "robot")
            {
                // extract name,filename, xyz and rpy from the node.
                name = _node.attribute("name").value();
                std::cout << "robot name : " << name << std::endl;
                filename = _node.child("path").first_attribute().value();
                
                Ogre::StringConverter::parse(_node.child("origin").attribute("xyz").value(),_xyz);
                Ogre::StringConverter::parse(_node.child("origin").attribute("rpy").value(), _rpyV); 
                
                convertOgreVecToEigen(_xyz, xyz);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.x),Ogre::Vector3::UNIT_X);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.y),Ogre::Vector3::UNIT_Y);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.z),Ogre::Vector3::UNIT_Z);
                convertOgreQuatToEigen(_rpy,rpy);

                // TO-DO: scale to be set in xml file later.
                scale = scale.Ones();

                std::cout << "xyz: " << xyz << std::endl;
                std::cout << "rpy: " << rpy << std::endl;

                createRobotObject(name,filename,false,xyz,rpy);
                // setRobotMeshOrientation(name,-90,mviz::AXIS::X);
                
            }
            else if (_node_name == "object")
            {
                name = _node.attribute("name").value();
                type = _node.attribute("type").value();

                Ogre::StringConverter::parse(_node.child("origin").attribute("xyz").value(),_xyz);
                Ogre::StringConverter::parse(_node.child("origin").attribute("rpy").value(), _rpy); 
                
                convertOgreVecToEigen(_xyz, xyz);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.x),Ogre::Vector3::UNIT_X);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.y),Ogre::Vector3::UNIT_Y);
                _rpy = _rpy * Ogre::Quaternion(Ogre::Radian(_rpyV.z),Ogre::Vector3::UNIT_Z);
                convertOgreQuatToEigen(_rpy,rpy);

                if (type == "mesh")
                {
                    filename = _node.child("mesh").attribute("filename").value();
                    createGraphicalObject(filename,name,xyz,rpy,scale);
                }
                else if(type == "box")
                {
                    l = _node.child("dim").attribute("l").as_double();
                    b = _node.child("dim").attribute("b").as_double();
                    h = _node.child("dim").attribute("h").as_double();
                    std::cout << "creating box\n";
                    createBox(name,l,b,h);
                    setObjectPoseAndRotation(name,xyz,rpy);
                }
                else if(type == "cylinder")
                {
                    r = _node.child("dim").attribute("radius").as_double();
                    h = _node.child("dim").attribute("height").as_double();
                    std::cout << "creating cylinder\n";
                    createCylinder(name,r,h);
                    setObjectPoseAndRotation(name,xyz,rpy);
                }
                else if(type == "sphere")
                {
                    r = _node.child("dim").attribute("radius").as_double();
                    std::cout << "creating sphere\n";
                    createSphere(name,r);
                    setObjectPoseAndRotation(name,xyz,rpy);
                }       
                else
                {
                    
                }
                
            }
            else
            {
                /* code */
            }
            
            

        }
        
        
        
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

    void mGraphics::createRobotObject(std::string _robotName, std::string _robot_filename, bool _show_collision,
                                      Eigen::Vector3d _bpos ,Eigen::Quaterniond _brot)
    {
        // Check whether the filename extension is ".urdf".
        std::filesystem::path path(_robot_filename);

        // std::cout << "I am here\n";
        // _show_collision = true;
        // _show_collision = false;

        // assign base_pos and orientation.

        assert(path.extension() == ".urdf");
        // create a new SceneNode from root node and start building robot graphical object from it.
        Ogre::SceneNode* robot_root_node = scnMgr->getRootSceneNode()->createChildSceneNode();
        // create a new robot object.
        mRobot* robot_object = new mRobot(_robotName,_robot_filename,scnMgr,robot_root_node, _bpos, _brot,_show_collision);
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

    void mGraphics::setObjectPoseAndRotation(std::string _objName, Eigen::Vector3d _pose, Eigen::Quaterniond _qRot)
    {
        try
        {
            objects.at(_objName)->setPosition(Ogre::Vector3(_pose.x(),_pose.y(), _pose.z()));
            objects.at(_objName)->setRotation(Ogre::Quaternion(_qRot.w(),_qRot.x(),_qRot.y(),_qRot.z()));
        }
        catch(const std::exception& e)
        {
            std::cout << "SetObjectPoseAndRotation: No mObject with given name. Name: " << _objName << std::endl;
            std::cerr << e.what() << '\n';
            std::quick_exit(-1);
        }
        
    }

    void mGraphics::createGraphicalObject(std::string _fileName, std::string objName, Eigen::Vector3d _pos, Eigen::Quaterniond _qrot,
                                          Eigen::Vector3d _scale, std::string parent_frame)
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
            createMeshFromFile(_fileName,mesh_name);
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
        
        objPtr->attachChildMesh(scnMgr,mesh_name, Ogre::Vector3(0,0,0),Ogre::Quaternion(1,0,0,0),
                                            Ogre::Vector3(_scale.x(), _scale.y(), _scale.z()));   // assuming entity name and mesh name as same at this momemt.
        // set position and orientation.
        objSceneNode->setPosition(Ogre::Vector3(_pos.x(), _pos.y(), _pos.z()));
        objPtr->setPosition(_pos);
        objPtr->setRotation(_qrot.w(), _qrot.x(), _qrot.y(), _qrot.z());
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

    void mGraphics::createBox(std::string _name, float l, float b, float h, std::string parent)
    {
        std::string mesh_file = "mCube.mesh";
        mObject* object = new mObject();
        Ogre::SceneNode* _node;

        if (parent == "")
        {
            object->setSceneNode(scnMgr->getRootSceneNode()->createChildSceneNode());
        }
        else
        {
            try
            {
                object->setSceneNode(objects.at(parent)->getSceneNode()->createChildSceneNode());
            }
            catch(const std::exception& e)
            {
               std::cerr << e.what() << std::endl;
               throw std::runtime_error("Error while creating box. Invalid parent name\n");
            }
            
        }
        // push the object to list.
        objects[_name] = object;
        object->attachChildMesh(scnMgr,mesh_file,Ogre::Vector3(0),Ogre::Quaternion(1,0,0,0),Ogre::Vector3(l,b,h));
        
    }

    void mGraphics::createCylinder(std::string _name, float r, float h, std::string parent)
    {
        std::string mesh_file = "mCylinder.mesh";
        mObject* object = new mObject();
        Ogre::SceneNode* _node;

        if (parent == "")
        {
            object->setSceneNode(scnMgr->getRootSceneNode()->createChildSceneNode());
        }
        else
        {
            try
            {
                object->setSceneNode(objects.at(parent)->getSceneNode()->createChildSceneNode());
            }
            catch(const std::exception& e)
            {
               std::cerr << e.what() << std::endl;
               throw std::runtime_error("Error while creating box. Invalid parent name\n");
            }
            
        }
        // push the object to list.
        objects[_name] = object;
        object->attachChildMesh(scnMgr,mesh_file,Ogre::Vector3(0),Ogre::Quaternion(1,0,0,0),Ogre::Vector3(r,r,h/10));
    }

    void mGraphics::createSphere(std::string _name, float r, std::string parent)
    {
        std::string mesh_file = "mSphere.mesh";
        mObject* object = new mObject();
        Ogre::SceneNode* _node;

        if (parent == "")
        {
            object->setSceneNode(scnMgr->getRootSceneNode()->createChildSceneNode());
        }
        else
        {
            try
            {
                object->setSceneNode(objects.at(parent)->getSceneNode()->createChildSceneNode());
            }
            catch(const std::exception& e)
            {
               std::cerr << e.what() << std::endl;
               throw std::runtime_error("Error while creating box. Invalid parent name\n");
            }
            
        }
        // push the object to list.
        objects[_name] = object;
        object->attachChildMesh(scnMgr,mesh_file,Ogre::Vector3(0),Ogre::Quaternion(1,0,0,0),Ogre::Vector3(r,r,r));
    }

    void mGraphics::setObjectColor(std::string _Objname,float r, float g, float b)
    {
        Ogre::ColourValue _color = Ogre::ColourValue(r,g,b);
        Ogre::SceneNode* _node;
        std::string mesh_name;
        std::vector<std::string> _mesh_list;
        _mesh_list.push_back("mCube.mesh");
        _mesh_list.push_back("mCylinder.mesh");
        _mesh_list.push_back("mSphere.mesh");
        // search the node.
        try
        {
            for (int i = 0; i < 3; i++)
            {
                _node = getGraphicalObject(_Objname)->getChildMeshNode(_mesh_list[i]);
                if (_node != nullptr)
                {
                    mesh_name = _mesh_list[i];
                    break;
                }
            }
            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "Inside setObjectColor function" << std::endl;
        }
        
        // add color to the object.
        Ogre::MaterialPtr pmat = Ogre::MaterialManager::getSingleton().getByName(_Objname);
        if (pmat == nullptr)
        {
            std::cout << "creating new material " << std::endl;
            pmat = Ogre::MaterialManager::getSingleton().create(_Objname,"UserData");
        }
        
        pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
        pmat->getTechnique(0)->getPass(0)->setDiffuse(_color*0);
        pmat->getTechnique(0)->getPass(0)->setSpecular(_color*0);
        
        Ogre::Entity* ent = static_cast<Ogre::Entity*>(_node->getAttachedObject(0));
        ent->getSubEntity(0)->setMaterialName(_Objname,"UserData");
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

    mGraphics::~mGraphics()
    {
        for ( auto robot: robots)
        {
            delete robot.second;
        }
        std::cout << "Deleted all mRobot objects" << std::endl;
        
        for (auto _object: objects)
        {
            delete _object.second;
        }
        std::cout << "Deleted all mObject objects" << std::endl;
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
        options["vsync"] = true;   // to deal with FPS being stuck to refresh rate of monitor.
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
        scnMgr->setAmbientLight(Ogre::ColourValue(0.9, 0.5, 0.5));
        // scnMgr->setAmbientLight(Ogre::ColourValue(1.0, 1.0, 1.0));

        // set newlight.

        Ogre::Light* light = scnMgr->createLight("MainLight1", Ogre::Light::LightTypes::LT_POINT);
        
        // Ogre::Light* light = scnMgr->createLight("MainLight1", Ogre::Light::LightTypes::LT_DIRECTIONAL);

        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->attachObject(light);
        lightNode->setPosition(10, 10, 1000);
        lightNode->setDirection(-1, -1, -1);

        // another light node.
        Ogre::Light* light2 = scnMgr->createLight("MainLight2");
        Ogre::SceneNode* lightNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode2->attachObject(light2);

        lightNode2->setPosition(-100, -10, -500);
        lightNode->setDirection(0, 0, 1);
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
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/merai/Files/C++/viz/resources",
                                                                        "FileSystem","UserData",true);
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/merai/Files/C++/viz/resources/meshes",
                                                                        "FileSystem","UserData",true);
        // now initialise the resourcegroup.
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("UserData");
        std::cout << "Finished setup" << std::endl;

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
            createMeshFromFile(_FilePath,mesh_name);
        }

        // create Entity.
        Ogre::Entity* ogreEntity = scnMgr->createEntity(mesh_name);
        // Ogre::MeshPtr mptr = ogreEntity->getMesh();
        
        Ogre::SceneNode* ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        ogreNode->attachObject(ogreEntity);
        ogreNode->setPosition(pos);
        
    }

    void mGraphics::createLine(std::vector<Eigen::Vector3d> _points,Eigen::Vector3d _color, float _lineWidth,
                            bool _connected,std::string _frameObject)
    {
        Ogre::SceneNode* _node;
        if (_frameObject == "")
        {
            _node  = scnMgr->getRootSceneNode();
        }
        else
        {
            try
            {
                _node = objects.at(_frameObject)->getSceneNode();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                throw std::runtime_error("CreateLine: Unable to find frameobject with give name: " + _frameObject );
            }
        
        }
        _count++;
        Ogre::ManualObject* man = scnMgr->createManualObject("line"+std::to_string(_count));

        Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("Line","UserData");
        mat->getTechnique(0)->getPass(0)->setLineWidth(_lineWidth);
        mat->getTechnique(0)->getPass(0)->setAmbient(_color.x(),_color.y(),_color.z());
        mat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(_color.x(),_color.y(),_color.z()));

        if (_connected)
            man->begin(mat,Ogre::RenderOperation::OT_LINE_STRIP);
        else
            man->begin(mat, Ogre::RenderOperation::OT_LINE_LIST);

        assert(("CreateLine: _points vector must have at least 2 Eigen vectors to draw a line",_points.size() >=2));
        for (unsigned int i = 0; i < _points.size(); i++)
        {
            man->position(_points[i].x(),_points[i].y(),_points[i].z());
            man->colour(_color.x(),_color.y(),_color.z());
        }
        man->end();
        // attach the manual object to the scenenode.
        _node->attachObject(man);
        std::cout << "Lines created" << std::endl;    
       
    }

    void mGraphics::createPoints(std::vector<Eigen::Vector3d>& _points, Eigen::Vector3d _color, float _pointSize,
        std::string _frameObject)
    {
        Ogre::SceneNode* _node;
        if (_frameObject == "")
        {
            _node  = scnMgr->getRootSceneNode();
        }
        else
        {
            try
            {
                _node = objects.at(_frameObject)->getSceneNode();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                throw std::runtime_error("CreatePoints: Unable to find frameobject with give name: " + _frameObject );
            }
        }
        _count++;
        Ogre::ManualObject* man = scnMgr->createManualObject("points_"+std::to_string(_count));

        Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("points","UserData");
        if (mat.get() == nullptr)
            throw std::runtime_error("Material could not be found. Inside createPoints function.");
        mat->getTechnique(0)->getPass(0)->setPointSpritesEnabled(false);
        mat->getTechnique(0)->getPass(0)->setPointSize(_pointSize);
        mat->getTechnique(0)->getPass(0)->setAmbient(_color.x(),_color.y(),_color.z());
        mat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(_color.x(),_color.y(),_color.z()));

        man->begin(mat,Ogre::RenderOperation::OT_POINT_LIST);
        for (unsigned int i = 0; i < _points.size(); i++)
        {
            man->position(_points[i].x(),_points[i].y(),_points[i].z());
            man->colour(_color.x(),_color.y(),_color.z());
        }
        man->end();
        // attach the manual object to the scenenode.
        _node->attachObject(man);
        std::cout << "Points created" << std::endl;  
    }

    // ....................... functions not part of Graphics object........................ //
    void createMeshFromFile(std::string filepath, Ogre::String& MeshName)
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

    void convertOgreVecToEigen(Ogre::Vector3& _ogV, Eigen::Vector3d& _eV)
    {
        _eV[0] = _ogV.x;
        _eV[1] = _ogV.y;
        _eV[2] = _ogV.z;
    }

    void convertOgreQuatToEigen(Ogre::Quaternion& _ogQuat, Eigen::Quaterniond& _eQuat)
    {
        _eQuat.w() = _ogQuat.w;
        _eQuat.x() = _ogQuat.x;
        _eQuat.y() = _ogQuat.y;
        _eQuat.z() = _ogQuat.z;
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
        ImGui::SetNextWindowSize(ImVec2(300,10));
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





