#include <sim.h>
int main(int argc, char const *argv[])
{
    Simulation sim;
    std::cout << "Bullet test example\n";
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0,0, -9.81));

    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(50,50,1));
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0,0,0));

        // set the ground to be static i.e. mass = 0;
        btScalar mass(0.);
        btVector3 localInertia(0, 0, 0);
        bool isDynamic = (mass != 0.f);
        if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        // construct a rigid body.
        btRigidBody::btRigidBodyConstructionInfo rbinfo(mass,myMotionState,groundShape,localInertia);
        btRigidBody* body = new btRigidBody(rbinfo);

        dynamicsWorld->addRigidBody(body);
    }

    {
        // create another rigid body.
        btCollisionShape* colShape = new btSphereShape(btScalar(1.));
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);
        bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(2, 1, 10));

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(body);

    }

    for (int i = 0; i < 20; i++)
    {
        dynamicsWorld->stepSimulation(1,10,0.1);
        for (int j = dynamicsWorld->getNumCollisionObjects()-1; j >= 0; j--)
        {
            btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
            // btRigidBody* body = btRigidBody::upcast(obj);
            btRigidBody* body = dynamic_cast<btRigidBody*>(obj);
            btTransform trans;
            if (body && body->getMotionState())
            {
                body->getMotionState()->getWorldTransform(trans);
            }
            else
            {
                trans = body->getWorldTransform();
            }

            if (j == 1)
                std::cout << "Transform : \n " << trans.getOrigin().getX() << " "
                        << trans.getOrigin().getY() << " " << trans.getOrigin().getZ() << std::endl;
            
        }
        
        
    }
    std::cout << "Finished" << std::endl;
    
    return 0;
}
