#include <mGraphics.h>
// #include <Logger.h>


bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    mviz::mVisualizer viz("Mviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();
    
    // Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/penguin.mesh";
    // Ogre::Vector3 v1(0.0, 0.0, 0.0);
    std::string robot_name = "Mamun";
    
    std::cout << "I am here\n";
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/jaxon_description/urdf/jaxon_jvrc_test.urdf");
    // viz.setRobotMeshOrientation(robot_name,-90,mviz::AXIS::X);

    // while (runloop)
    // {
    //     if (!viz.RenderOneFrame())
    //         break;
    // }
    

    // viz.addEntity(filePath,v1);

    std::cout << "Font from here\n";
    
    viz.loadFont();

    std::cout << "Entering Render loop " << std::endl;

    while (runloop)
    {
        if (!viz.ImguiRenderOneFrame())
            break;
    }
    

    return 0;
}
