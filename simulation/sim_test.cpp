// #include <sim.h>
// int main(int argc, char const *argv[])
// {
//     Simulation sim;
//     std::cout << "Bullet test example\n";
//     btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

//     btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

//     btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

//     btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

//     btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

//     dynamicsWorld->setGravity(btVector3(0,0, -9.81));

//     {
//         btCollisionShape* groundShape = new btBoxShape(btVector3(50,50,1));
//         btTransform groundTransform;
//         groundTransform.setIdentity();
//         groundTransform.setOrigin(btVector3(0,0,0));

//         // set the ground to be static i.e. mass = 0;
//         btScalar mass(0.);
//         btVector3 localInertia(0, 0, 0);
//         bool isDynamic = (mass != 0.f);
//         if (isDynamic)
// 			groundShape->calculateLocalInertia(mass, localInertia);
//         btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
//         // construct a rigid body.
//         btRigidBody::btRigidBodyConstructionInfo rbinfo(mass,myMotionState,groundShape,localInertia);
//         btRigidBody* body = new btRigidBody(rbinfo);

//         dynamicsWorld->addRigidBody(body);
//     }

//     {
//         // create another rigid body.
//         btCollisionShape* colShape = new btSphereShape(btScalar(1.));
//         btTransform startTransform;
//         startTransform.setIdentity();

//         btScalar mass(1.f);
//         bool isDynamic = (mass != 0.f);

// 		btVector3 localInertia(0, 0, 0);
// 		if (isDynamic)
// 			colShape->calculateLocalInertia(mass, localInertia);

// 		startTransform.setOrigin(btVector3(2, 1, 10));

//         btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
// 		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
// 		btRigidBody* body = new btRigidBody(rbInfo);

// 		dynamicsWorld->addRigidBody(body);

//     }

//     for (int i = 0; i < 20; i++)
//     {
//         dynamicsWorld->stepSimulation(1,10,0.1);
//         for (int j = dynamicsWorld->getNumCollisionObjects()-1; j >= 0; j--)
//         {
//             btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
//             // btRigidBody* body = btRigidBody::upcast(obj);
//             btRigidBody* body = dynamic_cast<btRigidBody*>(obj);
//             btTransform trans;
//             if (body && body->getMotionState())
//             {
//                 body->getMotionState()->getWorldTransform(trans);
//             }
//             else
//             {
//                 trans = body->getWorldTransform();
//             }

//             if (j == 1)
//                 std::cout << "Transform : \n " << trans.getOrigin().getX() << " "
//                         << trans.getOrigin().getY() << " " << trans.getOrigin().getZ() << std::endl;
            
//         }
        
//         break;
//     }

//     // std::string _filename = "/home/asp/Downloads/sphere/sphere-cubecoords.obj";
//     std::string _filename = "/home/asp/Files/resources/urdf_files_dataset/urdf_files/oems/qarm_quanser/qarm_description/meshes/base_link.STL";
//     // std::string _filename = "/home/asp/Files/resources/urdf_files_dataset/urdf_files/matlab/valkyrie/urdf/urdf/model/meshes/torso/torso.dae";
//     sim.addBodyFromFile("m1",_filename,Eigen::Vector3d(0,7,0));
//     // sim.addBodyFromFile("m1","/home/asp/Downloads/sibenik/sibenik.obj",Eigen::Vector3d(0,7,0));

//     std::cout << "Finished" << std::endl;
    
//     return 0;
// }

#include <simDebugViewer.h>
#include <simMultiBody.h>

int main(int argc, char const *argv[])
{
    std::string _urdf_file_name = "/home/asp/Files/cpp/projects/viz/src/kuka.urdf";
    _urdf_file_name = "/home/asp/Files/resources/urdf_files_dataset/urdf_files/oems/anymal_anybotics/anymal_b_simple_description/urdf/anymal.urdf";
    // _urdf_file_name = "/home/asp/Files/resources/baxter_common/baxter_description/urdf/baxter.urdf";
    // _urdf_file_name = "/home/asp/Files/resources/urdf_files_dataset/urdf_files/oems/xacro_generated/franka_emika/franka_description/robots/dual_panda/dual_panda.urdf";
    // _urdf_file_name = "/home/asp/Files/sai2/OpenSai/core/sai2-model/urdf_models/panda/panda_arm.urdf";
    // _urdf_file_name = "/home/asp/Files/resources/baxter_common/baxter_description/urdf/baxter.urdf";
    // _urdf_file_name = "/home/asp/Files/sai2/OpenSai/core/sai2-model/urdf_models/iiwa7/kuka_iiwa.urdf";
    // _urdf_file_name = "/home/asp/Files/sai2/OpenSai/core/sai2-model/urdf_models/HRP4C/HRP4C_custom_v1.urdf";
    simMultiBodyDynamicsWorld* sim = new simMultiBodyDynamicsWorld();
    sim->InitialiseDynamicsWorld();
    sim->LoadRobotFromURDFFile(_urdf_file_name);
    
    RobotObject* _robot = sim->getMultiBodyObject(0);
    
    btMultiBody* p_multibody = sim->getRobotObject(0);
    Eigen::VectorXd _jpos(p_multibody->getNumDofs());
    sim->getRobotJointPos(0,_jpos);
    // std::cout << _jpos  << std::endl;
    
    // for (int i = 0; i < p_multibody->getNumLinks(); i++)
    // {
    //     std::cout << "Mass: " << p_multibody->getLinkMass(i) << std::endl;
    //     std::cout << p_multibody->isLinkStaticOrKinematic(i) << std::endl;
    //     std::cout << "_________________" << std::endl;
    // }

    mMultiBody* mt = sim->getMultiBodyObject(0);
    std::cout << "Using jointNameIndexList" << std::endl;
    for (auto it : mt->_jointNameIndexList)
    {
        std::cout << it.first << " " << it.second << std::endl;
    }
    
    simDebugViewer v(sim);
    v.renderViewer();
    delete sim;
    return 0;
}
