
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreCameraMan.h"
#include "OgreResourceGroupManager.h"
#include "OgreCodec.h"
#include <iostream>
#include <filesystem>
#include "mGraphics.h"

// C - headers
#include <sys/stat.h>
#include <signal.h>

using namespace Ogre;
using namespace OgreBites;

void AppInfo(std::string str);
void getNodeInfo(Ogre::SceneNode* Node);

struct AssOptions
{
    Ogre::String source;
    Ogre::String dest;
    Ogre::String logFile;

    Ogre::BinaryOptionList options;

    AssOptions() { logFile = "OgreAssimp.log"; };
};


class BasicTutorial1
        : public ApplicationContext
        , public InputListener
{
public:
    bool loopEnd = false;
    Ogre::SceneNode* cNode;
    OgreBites::CameraMan* mCameraMan;
    OgreBites::ApplicationContext ctx;
    Ogre::Viewport* vp;
    BasicTutorial1();
    virtual ~BasicTutorial1() {}

    void addEntity(Ogre::SceneManager* scm);
    void addEntityFromFile(Ogre::SceneManager* scm, Ogre::String filename);
    void addEntityFromImportedFile(Ogre::SceneManager* scm, Ogre::String filename);
    void moveInCircle(Ogre::SceneNode* Node);

    void setup();
    bool keyPressed(const KeyboardEvent& evt);
};


BasicTutorial1::BasicTutorial1()
    : ApplicationContext("OgreTutorialApp")
{
}


void BasicTutorial1::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();
    addInputListener(this);

    // get a pointer to the already created root
    Root* root = getRoot();
    SceneManager* scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);
    
    // -- tutorial section start --
    //! [turnlights]
    scnMgr->setAmbientLight(ColourValue(1.0, 1.0, 1.0));
    //! [turnlights]

    //! [newlight]
    Light* light = scnMgr->createLight("MainLight1");
    SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    //! [newlight]

    //! [lightpos]
    lightNode->setPosition(20, 20, 50);
    //! [lightpos]

    // create more lights.
    // Light* light2 = scnMgr->createLight("Mainlight2");
    // SceneNode* lightNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
    // lightNode2->attachObject(light2);
    // lightNode2->setPosition(0,0,-50);
    
    // Light* light3 = scnMgr->createLight("Mainlight3");
    // SceneNode* lightNode3 = scnMgr->getRootSceneNode()->createChildSceneNode();
    // lightNode3->attachObject(light3);
    // lightNode3->setPosition(0,50,0);

    // Light* light4 = scnMgr->createLight("Mainlight4");
    // SceneNode* lightNode4 = scnMgr->getRootSceneNode()->createChildSceneNode();
    // lightNode4->attachObject(light4);
    // lightNode4->setPosition(0,-50,0);

    //! [camera]
    SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

    // create the camera
    Camera* cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(0.05); // specific to this sample
    cam->setAutoAspectRatio(true);
    // cam->setFOVy(Ogre::Radian(0.08));
    camNode->attachObject(cam);
    camNode->setPosition(0, 0, 10);

    AppInfo("setting camNode to CameraMan");
    mCameraMan = new OgreBites::CameraMan(camNode);
    mCameraMan->setTopSpeed(10);
    mCameraMan->setStyle(OgreBites::CameraStyle::CS_ORBIT);
    addInputListener(mCameraMan);
    AppInfo("Completed setting.");
    
    // and tell it to render into the main window
    Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
    vp->setBackgroundColour(Ogre::ColourValue(0,0.2,0.2));
    //! [camera]

    std::cout << "calling addEntity \n";
    // addEntity(scnMgr);
    // addEntityFromFile(scnMgr,"/home/asp/Downloads/roadBike/roadBike.mesh");
    // addEntityFromImportedFile(scnMgr,"/home/asp/Files/cpp/projects/franka_ros/franka_description/meshes/visual/link0.dae");
    Ogre::String ent_name;
    mviz::creatMeshFromFile("/home/asp/Downloads/bearded-guy-obj/Bearded_guy.obj",
                            ent_name);
    Ogre::Entity* ogreEntity = scnMgr->createEntity(ent_name);

    Ogre::SceneNode* ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode->attachObject(ogreEntity);
    cNode = ogreNode;
    
    std::cout << "exiting addEntity \n";
    // //! [entity1]
    // Entity* ogreEntity = scnMgr->createEntity("ogrehead.mesh");
    // //! [entity1]

    // //! [entity1node]
    // SceneNode* ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    // //! [entity1node]

    // //! [entity1nodeattach]
    // ogreNode->attachObject(ogreEntity);
    //! [entity1nodeattach]

    // //! [cameramove]
    // camNode->setPosition(0, 47, 222);
    // //! [cameramove]

    // -- tutorial section end --
}


void BasicTutorial1::addEntity(Ogre::SceneManager* scm)
{
    Entity* ogreEntity = scm->createEntity("ogrehead.mesh");
    std::cout << "created entity.\n";
    SceneNode* ogreNode = scm->getRootSceneNode()->createChildSceneNode();
    std::cout << "created child scene node\n";
    ogreNode->attachObject(ogreEntity);
    std::cout << "Attached entity to node\n";
}

void BasicTutorial1::addEntityFromFile(Ogre::SceneManager*scm, Ogre::String filename)
{
    // Ogre::String source = filename;
    // FILE* pFile = fopen(source.c_str(),"rb");
    // if (!pFile)
    //     OGRE_EXCEPT(Exception::ERR_FILE_NOT_FOUND,"File " + source + " not found.", "OgreMeshLoaded");
    // struct stat tagStat;
    // stat(source.c_str(),&tagStat);
    // Ogre::MemoryDataStream* memstream = new Ogre::MemoryDataStream(tagStat.st_size,true,true);
    // fread((void*) memstream->getPtr(), tagStat.st_size,1024,pFile);
    // fclose(pFile);

    // Ogre::MeshPtr pMesh = Ogre::MeshManager::getSingleton().createManual("test_1",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    // Ogre::MeshSerializer meshSerializer;
    // Ogre::DataStreamPtr stream(memstream);
    // meshSerializer.importMesh(stream,pMesh.get());


    // Entity* ogreEntity = scm->createEntity("t1","test_1");
    // std::cout << "created entity.\n";
    // ogreEntity->setMaterialName("/home/asp/Downloads/roadBike/roadBike.material");

    Ogre::ResourceGroupManager::getSingleton().createResourceGroup("custom");

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/asp/Downloads/roadBike","FileSystem","custom");
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("custom");
    Ogre::ResourceGroupManager::getSingleton().loadResourceGroup("custom");

    Entity* ogreEntity = scm->createEntity("roadBike.mesh");
    // ogreEntity->getVisible();
    // ogreEntity->setMaterialName("roadBike");
    
    SceneNode* ogreNode = scm->getRootSceneNode()->createChildSceneNode();
    std::cout << "created child scene node\n";
    ogreNode->attachObject(ogreEntity);
    std::cout << "Attached entity to node\n";
}

void BasicTutorial1::addEntityFromImportedFile(Ogre::SceneManager* scm, Ogre::String filepath)
{
    if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("User"))
    {
        Ogre::ResourceGroupManager::getSingleton().createResourceGroup("User");
    }
    // traverse through the directory of the file and add all the 
    // subdirectory paths to a new resource group.
    // std::filesystem::path path = filepath;
    std::filesystem::path path = std::filesystem::canonical(filepath);
    std::cout << path.parent_path() << std::endl;
    std::filesystem::recursive_directory_iterator it(path.parent_path());
    std::filesystem::recursive_directory_iterator end;

    // Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().string(),"FileSystem","User");

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().generic_string(),"FileSystem","User");

    while (it != end)
    {
        if (it->is_directory())
        {
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(it->path().string(),"FileSystem","User");
        }   
        ++it;
    }

    std::cout << "Intilising resource group User" << std::endl;
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("User");
    std::cout << "loading resource group User " << std::endl;
    Ogre::ResourceGroupManager::getSingleton().loadResourceGroup("User");
    

    Ogre::String basename, ext, pathname;
    Ogre::StringUtil::splitFullFilename(filepath, basename, ext, pathname);

    std::cout << "Creating Codec object." << std::endl;
    auto codec = Codec::getCodec(ext);

    AssOptions opts;
    opts.source = basename + "." + ext;

    Ogre::MeshPtr mesh = MeshManager::getSingleton().createManual(basename + "." + ext, "User");
    mesh->getUserObjectBindings().setUserAny("_AssimpLoaderOptions", opts.options);
    std::cout << "Decoding the obj file to mesh" << std::endl;
    codec->decode(Root::openFileStream(opts.source), mesh.get());

    std::cout << "creating an Entity out of the decoded mesh." << std::endl;
    Ogre::Entity* ogreEntity = scm->createEntity(basename + "." + ext);

    std::cout << "successsfully created the Entity." << std::endl;
    SceneNode* ogreNode = scm->getRootSceneNode()->createChildSceneNode();
    std::cout << "created child scene node\n";
    ogreNode->attachObject(ogreEntity);
    std::cout << "Attached entity to node\n";
}

void BasicTutorial1::moveInCircle(Ogre::SceneNode* Node)
{
    static Ogre::Vector3 pose;
    Ogre::Radian dt = Ogre::Radian(0.00000);
    pose = pose + Ogre::Vector3(Ogre::Math::Sin(dt),Ogre::Math::Cos(dt), 0);
    Node->setPosition(pose);
}

bool BasicTutorial1::keyPressed(const KeyboardEvent& evt)
{
    if (evt.keysym.sym == SDLK_ESCAPE)
    {
        std::cout << "Detected ESC key. Exiting...\n";
        getRoot()->queueEndRendering();
        loopEnd = true;
    }

    else if (evt.keysym.sym == SDLK_PAGEDOWN)
    {
        std::cout << "Quitting render loop.\n";
        loopEnd = true;
    }

    else
    {
        std::cout << "Key pressed: " << evt.keysym.sym << std::endl;
        // mCameraMan->keyPressed(evt);
    }
    
    return true;
}




int main(int argc, char **argv)
{
    try
    {
    	BasicTutorial1 app;
        app.initApp();
        // app.getRoot()->startRendering();
        while (!app.loopEnd)
        {
            if (app.loopEnd)
            {
                std::cout << "LoopEnd: " << app.loopEnd << std::endl;
            }
            app.getRoot()->renderOneFrame();
        
        }
        
        
        app.closeApp();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error occurred during execution: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

void AppInfo(std::string str)
{
    std::cout << str << std::endl; 
}

void getNodeInfo(Ogre::SceneNode* Node)
{
    std::cout << Node->getPosition() << std::endl;
}