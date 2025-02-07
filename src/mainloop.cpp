// #include <mCommon.h>

#include <mGraphics.h>
#include <chrono>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    // attach a signal handler.
    signal(SIGINT,sighandler);

    mviz::mGraphics graphics("Mviz");
    std::string robot_name = "robot";
    Ogre::Vector3 v(0.0, 0.0, 100.0);
    Ogre::Vector3 v1(0.0, 0.0, 0.0);
    Ogre::String filePath = "/home/asp/Files/cpp/projects/ogre/Samples/Media/models/penguin.mesh";
    Ogre::String filePath1 = "/home/asp/Downloads/sponza/sponza.obj";


    graphics.attachFlagVariable(&runloop);
    graphics.initApp();
    // graphics.addEntity(filePath,v);
    // graphics.addEntity(filePath1,v1);
    // graphics.addEntity(argv[1],v1);
    graphics.createRobotObject(robot_name,"/home/asp/Files/cpp/projects/viz/src/kuka.urdf");
    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/anymal_b_simple_description/urdf/anymal_test.urdf");

    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/icub-models-master/iCub/robots/iCubGazeboV2_5_KIT_007/model_test.urdf");
    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/jaxon_description/urdf/jaxon_jvrc_test.urdf");
    // graphics.setBasePoseAndRotation(robot_name,Eigen::Vector3d(1,0,0),Eigen::Quaterniond::Identity());
    // graphics.creatGraphicalObject("ninja.mesh","head",Eigen::Vector3d(0,0,-100), Eigen::Quaterniond(1,0,0,0));
    graphics.createDynamicMeshObject("dm",Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));

    // bool ret;
    // int count = 0,n;
    // n = graphics.getRobotObject(robot_name)->getRobotNumJoints();
    // Eigen::VectorXd v2(n) ;
    // v2.setZero();
    // // v2.setOnes();

    // Eigen::Vector3d v3;
    // v3.setZero();
   
    // graphics.getRoot()->startRendering();
    while (runloop)
    {
        if (!graphics.RenderOneFrame())
            break;
        // graphics.updateRobotGraphics(robot_name,v2);
        // graphics.updateRobotGraphics("panda1",v2,v3,Eigen::Quaterniond::Identity());
        // auto t1 = std::chrono::high_resolution_clock::now();
        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto t = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
        // std::cout << t.count() << std::endl;
        // v3(2) = v3(2) + 0.001;
        // v2(0) = v2(0) + 0.01;
        // v2(3) = v2(3) + 0.01;
    }
    graphics.closeGraphics();

    return 0;
}
