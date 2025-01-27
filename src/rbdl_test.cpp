// #include <iostream>
// #include "rbdl/rbdl.h"
// #include "rbdl/rbdl_utils.h"
// #include "rbdl/addons/urdfreader/urdfreader.h"

// using namespace RigidBodyDynamics;
// using namespace RigidBodyDynamics::Math;

// int main(int argc, char const *argv[])
// {
//     std::string filepath = "/home/asp/Files/cpp/projects/viz/src/kuka.urdf";

//     Model* model = new Model();

//     if (!Addons::URDFReadFromFile(filepath.c_str(),model,false))
//     {
//         std::cout << "Error while loading the model." << std::endl;
//         abort();
//     }
    


//     return 0;
// }


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

int main(int argc, char const *argv[])
{
    Eigen::Affine3d A;
    A.setIdentity();

    std::cout << "A\n " << A.matrix() << std::endl;

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ());
    std::cout << "M1: \n" << m << std::endl;
    m = Eigen::AngleAxisd(1.57, Eigen::Vector3d(0,0,1));
    std::cout << "M2: \n" << m.matrix() << std::endl;
    return 0;
}
