// #include <mCommon.h>

#include <mGraphics.h>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    // attach a signal handler.
    signal(SIGINT,sighandler);

    mviz::mGraphics graphics("Mviz");
    Ogre::Vector3 v(0.0, 0.0, 100.0);
    Ogre::Vector3 v1(0.0, 0.0, 0.0);
    Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/penguin.mesh";
    Ogre::String filePath1 = "/home/asp/Downloads/sponza/sponza.obj";


    graphics.attachFlagVariable(&runloop);
    graphics.initApp();
    // graphics.addEntity(filePath,v);
    // graphics.addEntity(filePath1,v1);
    // graphics.addEntity(argv[1],v1);
    graphics.createRobotObject("panda","/home/asp/Files/cpp/projects/viz/src/kuka.urdf");

    bool ret;
    int count = 0;
    Eigen::VectorXd v2(7) ;
    v2.setZero();
    // graphics.getRoot()->startRendering();
    while (runloop)
    {
        if (!graphics.RenderOneFrame())
            break;
        graphics.updateRobotGraphics("panda",v2);
        v2(0) = v2(0) + 0.01;
        v2(3) = v2(3) + 0.03;
    }
    graphics.closeGraphics();

    return 0;
}
