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
class A
{
private:

protected:
    int a = 1;
public:
    A(/* args */) {};
    ~A() {};
    virtual void foo() {};
};

class B : public A
{
private:
    /* data */
public:
    int b = 2;
    B(/* args */) {a = b;};
    ~B() {};
};


int main(int argc, char const *argv[])
{
    A a1;
    B b1;
    A* ap1,ap2;
    ap1 = &a1;
    B* bp1,bp2;
    bp1 = &b1;

    bp1 = dynamic_cast<B*>(ap1);
    // ap2 = ap1;
    

    return 0;
}
