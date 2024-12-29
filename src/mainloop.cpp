#include <mCommon.h>
#include <mGraphics.h>

bool runloop = true;

int main(int argc, char const *argv[])
{
    mviz::mGraphics graphics;
    Ogre::Vector3 v(0.0, 0.0, 0.0);
    Ogre::String filePath = "/home/asp/Downloads/conference/conference.obj";

    graphics.attachFlagVariable(&runloop);
    graphics.initApp();
    graphics.addEntity(filePath,v);
    while (runloop)
    {
        graphics.RenderOneFrame();
    }
    graphics.closeGraphics();

    return 0;
}
