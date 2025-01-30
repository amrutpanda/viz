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

# include <iostream>

class boom
{
private:
    double pos = 1.2;
public:
    boom(/* args */) {};
    void setpos(double* _x) { pos = *_x;}
    ~boom(){};
};

int main(int argc, char const *argv[])
{
    boom* b = new boom;
    double t = 1.3;
    b->setpos(&t);

    for (int i = 0; i < 5; i++)
    {
        t = t + i;
        std::cout << t << std::endl;
    }
    

    return 0;
}
