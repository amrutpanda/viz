#include <mCommon.h>

#include <mGraphics.h>

bool runloop = true;

int main(int argc, char const *argv[])
{
    mviz::mGraphics graphics("My app 2");
    Ogre::Vector3 v(0.0, 0.0, 0.0);
    Ogre::Vector3 v1(0.0, 0.0, 0.0);
    Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/ninja.mesh";
    Ogre::String filePath1 = "/home/asp/Downloads/sponza/sponza.obj";


    graphics.attachFlagVariable(&runloop);
    graphics.initApp();
    // graphics.addEntity(filePath,v);
    // graphics.addEntity(filePath1,v1);
    graphics.addEntity(argv[1],v1);
    while (runloop)
    {
        graphics.RenderOneFrame();
    }
    graphics.closeGraphics();

    return 0;
}
