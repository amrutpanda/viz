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
    
    // Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/penguin.mesh";
    Ogre::Vector3 v1(0.0, 0.0, 0.0);

    
    std::string robot_name = "Mamun";
    
    // viz.readFile("myworld.world");
    // viz.createGraphicalObject(filePath,"penguin",Eigen::Vector3d(0,0,4),
    //                             Eigen::Quaterniond(0,0,0,0.5),Eigen::Vector3d(0.01,0.01,0.01));
    // viz.createGraphicalObject("/home/asp/Files/sai2/OpenSai/core/sai2-model/urdf_models/iiwa7/meshes/link0.obj","link5",
    //                 Eigen::Vector3d(0,0,4), Eigen::Quaterniond(0,0,0,0.5),Eigen::Vector3d(1,1,1) );
    
    // viz.createRobotObject(robot_name,"../src/kuka.urdf");
    viz.createRobotObject(robot_name,"/home/amrut/Files/resources/TCP-IP-ROS-6AXis/dobot_description/urdf/cr12_robot.urdf");
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/urdf_files_dataset/urdf_files/robotics-toolbox/val_description/model/robots/valkyrie_sim.urdf",true);
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/urdf_files_dataset/urdf_files/matlab/franka_description/robots/frankaEmikaPanda.urdf");
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/urdf_files_dataset/urdf_files/oems/anymal_anybotics/anymal_b_simple_description/urdf/anymal.urdf");
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/baxter_common/baxter_description/urdf/baxter.urdf");
    // viz.createRobotObject(robot_name,"/home/asp/Files/resources/urdf_files_dataset/urdf_files/oems/xacro_generated/franka_emika/franka_description/robots/dual_panda/dual_panda.urdf");
    // viz.setRobotMeshOrientation(robot_name,-90,mviz::AXIS::X);
    
    // viz.createSphere("1",0.4);

    std::vector<Eigen::Vector3d> _points;
    _points.push_back(Eigen::Vector3d(0.1,0.1,0.5));
    _points.push_back(Eigen::Vector3d(1.5,1.0, 0.8));
    _points.push_back(Eigen::Vector3d(2.5,1.0, 0.8));
    _points.push_back(Eigen::Vector3d(2.5,2.0, 0.8));

    // viz.createLine(_points,Eigen::Vector3d(1.0,0.5, 0.0),1.0, true);
    // viz.createPoints(_points,Eigen::Vector3d(0.5,0.0, 0.0),8.0);
    
    std::cout << "Font from here\n";
    
    viz.loadFont();

     // simulation instance.
    
    std::cout << "Entering Render loop " << std::endl;

    while (runloop)
    {
        if (!viz.ImguiRenderOneFrame())
            break;
            
    }
    
    viz.closeApp();

    return 0;
}
