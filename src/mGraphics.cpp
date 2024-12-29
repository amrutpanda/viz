
#include "mGraphics.h"

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

    const std::string& mGraphics::getName()
    {
        return urdf->getName();
    }

    void mGraphics::attachFlagVariable(bool* _flag)
    {
        flag = _flag;
    }

    void mGraphics::setup()
    {
        // do not forget to call the base.
        OgreBites::ApplicationContext::setup();
        addInputListener(this);

        // get a pointer to the already created root
        Ogre::Root* root = getRoot();
        scnMgr = root->createSceneManager();

        // register our scene with the RTSS
        Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
        shadergen->addSceneManager(scnMgr);

        // set ambient light.
        scnMgr->setAmbientLight(Ogre::ColourValue(1.0, 1.0, 1.0));

        // set newlight.
        Ogre::Light* light = scnMgr->createLight("MainLight1");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->attachObject(light);

        //! [lightpos]
        lightNode->setPosition(0, 0, 100);

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
        vp->setBackgroundColour(Ogre::ColourValue(0,0.2,0.2));
        //! [camera]
    }

    bool mGraphics::RenderOneFrame()
    {
        if (*flag)
        {
            return getRoot()->renderOneFrame();
        }
        else
        {
            return true;
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
            getRoot()->queueEndRendering();
        }
        return true;
    }

    void mGraphics::addEntity(std::string _FilePath, Ogre::Vector3& pos)
    {
        Ogre::String mesh_name;
        creatMeshFromFile(_FilePath,mesh_name);

        // create Entity.
        Ogre::Entity* ogreEntity = scnMgr->createEntity(mesh_name);
        Ogre::SceneNode* ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        ogreNode->attachObject(ogreEntity);
        ogreNode->setPosition(pos);

    }
    // ....................... functions not part of Graphics object........................ //
    void creatMeshFromFile(std::string filepath, Ogre::String& MeshName)
    {
        if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("User"))
        {
            Ogre::ResourceGroupManager::getSingleton().createResourceGroup("User");
        }
        std::filesystem::path path = std::filesystem::canonical(filepath);
        std::cout << path.parent_path() << std::endl;
        std::filesystem::recursive_directory_iterator it(path.parent_path());
        std::filesystem::recursive_directory_iterator end;

        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().generic_string(),
                                                                        "FileSystem","User");
        while (it != end)
        {
            if (it->is_directory())
            {
                Ogre::ResourceGroupManager::getSingleton().addResourceLocation(it->path().string(),"FileSystem","User");
            }   
            ++it;
        }

        // std::cout << "Intilising resource group User" << std::endl;
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("User");
        // std::cout << "loading resource group User " << std::endl;
        Ogre::ResourceGroupManager::getSingleton().loadResourceGroup("User");

        Ogre::String basename, ext, pathname;
        Ogre::StringUtil::splitFullFilename(filepath,basename,ext,pathname);

        std::cout << basename + "." + ext << std::endl;

        // std::cout << "Creating Codec object." << std::endl;
        auto codec = Ogre::Codec::getCodec(ext);

        AssOptions opts;
        opts.source = basename + "." + ext ;

        Ogre::MeshPtr m = Ogre::MeshManager::getSingleton().createManual(basename + "." + ext, "User");
        m->getUserObjectBindings().setUserAny("_AssimpLoaderOptions", opts.options);
        // std::cout << "Decoding the obj file to mesh" << std::endl;
        codec->decode(Ogre::Root::openFileStream(opts.source), m.get());

        // std::cout << "decoding complete." << std::endl;

        MeshName = basename + "." + ext;

    }


    void say_hello()
    {
        std::cout << "Hello. " << std::endl;
    }
} // namespace mviz




