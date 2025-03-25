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
    
    Eigen::Vector3d bp(0,2,0.1),sp(4,0,2.2);
    Eigen::Quaterniond bq,sq;

    
    // Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/penguin.mesh";
    // Ogre::Vector3 v1(0.0, 0.0, 0.0);
    std::string robot_name = "Mamun";
    
    viz.readFile("myworld.world");
    
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/jaxon_description/urdf/jaxon_jvrc_test.urdf");
    // viz.setRobotMeshOrientation(robot_name,-90,mviz::AXIS::X);
    
    // viz.createSphere("1",0.4);
    
    std::cout << "Font from here\n";
    
    viz.loadFont();

     // simulation instance.
    Simulation sim;
    sim.setGravity(0,0,-10);
    int gId = sim.addBodyBox("gr",50,50,0.01,0, Eigen::Vector3d(0,0,-0.01));
    int boxId = sim.addBodyBox("b1",0.2,0.2,0.2,2,Eigen::Vector3d(4,0.8,3.1),Eigen::Quaterniond(1,0.3,0.2,0.1));
    int spId = sim.addBodySphere("s1",0.2,0.01,Eigen::Vector3d(4,0,4.2),Eigen::Quaterniond(1,0.3,0.2,0.1));
    
    std::cout << "Entering Render loop " << std::endl;

    while (runloop)
    {
        if (!viz.ImguiRenderOneFrame())
            break;
            
            // running simulation methods inside this loop.
            // sim.applyForce(spId,Eigen::Vector3d(-0.001,0, 0));
            sim.stepSimulation(0.001);
            sim.getBodyPoseAndRotation(boxId,bp,bq);
            sim.getBodyPoseAndRotation(spId,sp,sq);
            
            // std::cout << bp << std::endl;
            // std::cout << sp << std::endl;
            // std::cout << "==================" << std::endl;
            
            viz.setObjectPoseAndRotation("box",bp,bq);
            viz.setObjectPoseAndRotation("sphere",sp,sq);
    }
    
    std::cout << "Closing graphics" << std::endl;
    viz.closeApp();
    std::cout << "Closing Simulation" << std::endl;
    sim.close();

    return 0;
}
