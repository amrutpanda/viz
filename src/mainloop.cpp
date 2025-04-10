// #include <mCommon.h>

#include <mGraphics.h>
#include <chrono>
#include <iostream>



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
    // graphics.createRobotObject(robot_name,"/home/asp/Files/cpp/projects/viz/src/kuka.urdf");
    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/anymal_b_simple_description/urdf/anymal_test.urdf");
    // while (runloop)
    // {
    //     if(!graphics.RenderOneFrame())
    //         break;
    // }
    
    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/icub-models-master/iCub/robots/iCubGazeboV2_5_KIT_007/model_test.urdf");
    // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/jaxon_description/urdf/jaxon_jvrc_test.urdf");
    // graphics.setBasePoseAndRotation(robot_name,Eigen::Vector3d(1,0,0),Eigen::Quaterniond::Identity());
    // graphics.creatGraphicalObject("ninja.mesh","head",Eigen::Vector3d(0,0,-100), Eigen::Quaterniond(1,0,0,0));
    // graphics.createDynamicMeshObject("dm",Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
    
    
    // graphics.setRobotMeshOrientation(robot_name,-90,mviz::AXIS::X);
    // bool ret;
    // int count = 0,n;
    // n = graphics.getRobotObject(robot_name)->getRobotNumJoints();
    // Eigen::VectorXd v2(n) ;
    // v2.setZero();
    // // v2.setOnes();
    
    // Eigen::Vector3d v3;
    // v3.setZero();
    graphics.createRobotObject(robot_name, "/home/asp/Files/cpp/projects/ogre/build/bullet3-3.25/examples/pybullet/gym/pybullet_data/plane.urdf");
    std::cout << "I am here\n";
    Eigen::Vector3d v3;
    v3.setZero();
    v3(2) = 1;
    Eigen::Quaterniond Q;
    Q.setIdentity();
    int count = 0;
    for (int i = 0; i < 10 ; i++)
    {
        for (size_t j = 0; j < 10; j++)
        {
            robot_name = "robot_"+std::to_string(i)+std::to_string(j);
            graphics.createRobotObject(robot_name, "/home/asp/Files/cpp/projects/viz/src/kuka.urdf",false,v3,Q);
            // graphics.createRobotObject(robot_name,"/home/asp/Files/resources/jaxon_description/urdf/jaxon_jvrc_test.urdf", v3, Q);
            // graphics.setRobotMeshOrientation(robot_name,-90,mviz::AXIS::X);
            // graphics.setBasePoseAndRotation(robot_name,v3,Q);
            // v3(0) = v3(0) + 0.5;
            v3(1) = v3(1) + 1.0;
            count++;
        }
        v3(0) = v3(0) + 1.0;
        v3(1) = 0;
        
    }
    
    int n = graphics.getRobotObject(robot_name)->getRobotNumJoints();
    Eigen::VectorXd v2(n);
    v2.setZero();
    // v2.setOnes();
    // graphics.getRoot()->startRendering();
    count = 0.01;
    while (runloop)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        if (!graphics.RenderOneFrame())
            break;
            count++;
            for (int i = 0; i < 10; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    robot_name = "robot_"+std::to_string(i)+std::to_string(j);
                    // std::cout << robot_name << std::endl;
                    v2.setZero();
                    v2(3) = count;
                    graphics.updateRobotGraphics(robot_name,v2);
                    // count = count + 0.1;
                }
                
            }
            
        auto t2 = std::chrono::high_resolution_clock::now();
        // v2(4) = count + 0.1;
        // std::cout << count << std::endl;
        // graphics.updateRobotGraphics(robot_name,v2);
        // graphics.updateRobotGraphics("panda1",v2,v3,Eigen::Quaterniond::Identity());
        // auto t1 = std::chrono::high_resolution_clock::now();
        // auto t2 = std::chrono::high_resolution_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
        // std::cout << 1000000/t.count() << std::endl;
        // v3(2) = v3(2) + 0.001;
        // v2(0) = v2(0) + 0.01;
        // v2(3) = v2(3) + 0.01;
    }
    graphics.closeGraphics();

    return 0;
}
