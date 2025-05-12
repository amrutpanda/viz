#include <mGraphics.h>
#include <sim.h>
#include <redisclient.h>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    RedisClient redisclient("127.0.0.1",6379);
    redisclient.connect();
    std::string _key = "q1";
    Eigen::Vector3d v(3);
    v << 1,12,2;
    redisclient.setEigenMatrix(_key,v);
    int count = 0;
    
    signal(SIGINT,sighandler);
    mviz::mVisualizer viz("Mviz");
    viz.attachFlagVariable(&runloop);
    viz.initApp();

    std::string robot_name = "Mamun";
    // viz.readFile("myworld.world");

    viz.loadFont();

    while (runloop)
    {
        if (!viz.ImguiRenderOneFrame())
            break;
        // if (!viz.RenderOneFrame())
        //     break;
        redisclient.setEigenMatrix(_key,v);
        std::cout << v << std::endl;
        v[2] = count;
        count++;
    }
    
    viz.closeApp();

    return 0;
}
