#include <mGraphics.h>
#include <sim.h>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    mviz::mVisualizer viz("Mviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();

    std::string robot_name = "Mamun";
    viz.readFile("myworld.world");

    viz.loadFont();

    while (runloop)
    {
        if (!viz.ImguiRenderOneFrame())
            break;
        // if (!viz.RenderOneFrame())
        //     break;
            
    }
    
    viz.closeApp();

    return 0;
}
