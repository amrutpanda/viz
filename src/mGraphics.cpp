
#include "mGraphics.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>

// #include <Ogre.h>
// #include <urdf_parser/urdf_parser.h>

namespace mviz
{
    void mGraphics::readFile(std::string FilePath)
    {
        urdf = urdf::parseURDFFile(FilePath);
        
        urdf::LinkConstSharedPtr root = urdf->getRoot();
        
        
    }

    urdf::ModelInterfaceSharedPtr mGraphics::getUrdfObject()
    {
        return urdf;
    }


    void mGraphics::urdf_to_ogre_converter(Ogre::SceneManager* scm)
    {
        // to be implemented.
        urdf::LinkSharedPtr link;

    }

    void mGraphics::createRobotObject(Ogre::SceneNode* _rNode, std::string _robotName, std::string _robot_filename)
    {

    }

    // Graphics related Methods.

    const std::string& mGraphics::getName()
    {
        return urdf->getName();
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
        lightNode->setPosition(0, 0, 100);

        // another light node.
        Ogre::Light* light2 = scnMgr->createLight("MainLight2");
        Ogre::SceneNode* lightNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode2->attachObject(light2);

        lightNode2->setPosition(-500, 100, 0);
        // another light node end.

        //! [camera]
        camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

        // create the camera
        Ogre::Camera* cam = scnMgr->createCamera("myCam");
        cam->setNearClipDistance(0.05); // specific to this sample
        cam->setAutoAspectRatio(true);
        // cam->setFOVy(Ogre::Radian(0.08));
        camNode->attachObject(cam);
        camNode->setPosition(0, 0, 10);

        // create a new Cameraman object and attach it to this object.
        mCameraMan = new OgreBites::CameraMan(camNode);
        mCameraMan->setTopSpeed(10);
        mCameraMan->setStyle(OgreBites::CameraStyle::CS_ORBIT);
        addInputListener(mCameraMan);

        // and tell it to render into the main window
        Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
        vp->setBackgroundColour(Ogre::ColourValue(0.0,1.0,1.0));
        //! [camera]

        // get RenderWindow;
        mWin = getRenderWindow();

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

        Ogre::MeshPtr m = Ogre::MeshManager::getSingleton().createManual(basename + "." + ext, "UserData");
        m->getUserObjectBindings().setUserAny("_AssimpLoaderOptions", opts.options);
        // std::cout << "Decoding the obj file to mesh" << std::endl;
        codec->decode(Ogre::Root::openFileStream(opts.source), m.get());

        // std::cout << "decoding complete." << std::endl;

        // MeshName = basename + "." + ext;
        MeshName = opts.source;

    }


    void say_hello()
    {
        std::cout << "Hello. " << std::endl;
    }
} // namespace mviz




