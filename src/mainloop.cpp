// #include <mCommon.h>

#include <mGraphics.h>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    // attach a signal handler.
    signal(SIGINT,sighandler);

    mviz::mGraphics graphics("Mviz");
    Ogre::Vector3 v(0.0, 0.0, 0.0);
    Ogre::Vector3 v1(0.0, 0.0, 0.0);
    Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/packs/DamagedHelmet/DamagedHelmet.mesh";
    Ogre::String filePath1 = "/home/asp/Downloads/sponza/sponza.obj";


    graphics.attachFlagVariable(&runloop);
    graphics.initApp();
    graphics.addEntity(filePath,v);
    // graphics.addEntity(filePath1,v1);
    // graphics.addEntity(argv[1],v1);

    bool ret;
    // graphics.getRoot()->startRendering();
    while (runloop)
    {
        if (!graphics.RenderOneFrame())
            break;
        
    }
    graphics.closeGraphics();

    return 0;
}
