#ifndef _COMMOM_RIGIDBODY_BASE_H
#define _COMMOM_RIGIDBODY_BASE_H

#include <simCommonHeaders.h>

class simCommonRigidBodyBase
{
private:
    
protected:
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btBroadphaseInterface* m_broadphase;
    btConstraintSolver* m_solver;
    public:
    // btDiscreteDynamicsWorld* m_dynamicsWorld;
    btMultiBodyDynamicsWorld* m_dynamicsWorld;
    simCommonRigidBodyBase(/* args */): m_collisionConfiguration(0), m_dispatcher(0),
                                        m_broadphase(0),m_solver(0),m_dynamicsWorld(0) {};
    virtual ~simCommonRigidBodyBase()
    {
        if (m_dynamicsWorld != nullptr)
        {
            delete m_dynamicsWorld;
        }
        // std::cout << "Hi 1\n";
        if (m_collisionShapes.size() != 0)
        {
            for (int i ; i < m_collisionShapes.size(); ++i)
            {
                if (m_collisionShapes[i] != nullptr)
                    delete m_collisionShapes[i];
            }
        }
        // std::cout << "Hi 2\n";
        if (m_collisionConfiguration != nullptr)
            delete m_collisionConfiguration;
        // std::cout << "Hi 3\n";
        if (m_dispatcher != nullptr)
            delete m_dispatcher;
        // std::cout << "Hi 4\n";
        if (m_broadphase != nullptr)
            // delete m_broadphase;
        // std::cout << "Hi 5\n";
        if (m_solver != nullptr)
            delete m_solver;
        std::cout << "cleared simCommonRigidBodyBase\n";
    }
    virtual void stepSimulation(float _timeStep)
    {
        if (m_dynamicsWorld)
        {
            m_dynamicsWorld->stepSimulation(btScalar(_timeStep));
        }
        
    }

    btDiscreteDynamicsWorld* getDynamicsWorld()
    {
        return m_dynamicsWorld;
    }
    
    void setGravity(double _gx, double _gy, double _gz)
    {
        m_dynamicsWorld->setGravity(btVector3(_gx,_gy,_gz));
    }

    void setGravity(btVector3& _gravity)
    {
        m_dynamicsWorld->setGravity(_gravity);
    }
};


// struct mMultiBody
// {
//     mMultiBody(): _multibody(0) {};
//     btMultiBody* _multibody;
//     std::string _name;
//     btAlignedObjectArray<btCollisionShape*> _colShapes;
//     btAlignedObjectArray<btMultiBodyLinkCollider*> _colliders;
//     std::unordered_map <std::string,int> _linkNameIndexMap;
//     std::unordered_map <std::string,int> _jointNameIndexMap;

//     // mMultiBody::mMultiBody()
//     // {
//     //     _multibody = 0;
//     // }
    
//     // mMultiBody::~mMultiBody()
//     // {
//     //     for (int i = 0; i < _colShapes.size(); i++)
//     //     {
//     //         delete _colShapes[i];
//     //         delete _colliders[i];
//     //     }
        
//     // }

//     void addMultiBody(btMultiBody* _pMultiBody)
//     {
//         _multibody = _pMultiBody;
//     }

// };


class mMultiBody
{
private:
    btAlignedObjectArray<btQuaternion> rot_world_to_local;
    btAlignedObjectArray<btVector3> local_origin_world_frame;
public:
    mMultiBody() {};
    ~mMultiBody()
    {
        delete _multibody;
        std::cout << "Freed bullet multiBody object" << std::endl;

        for (int i = 0; i < _jointFeedbackMap.size() ; i++)
        {
            if (_jointFeedbackMap[i] != nullptr)
            {
                delete _jointFeedbackMap[i];
            }
            
        }
        
        for(int i = 0 ; i < _colShapes.size(); ++i)
        {
            if (_colShapes[i] != nullptr)
            {
                if (_colShapes[i]->getUserPointer() != nullptr)
                {
                    btShapeHull* _hull = static_cast<btShapeHull*>(_colShapes[i]->getUserPointer());
                    delete _hull;
                }
                delete _colShapes[i];
            }
            if (_colliders[i] != nullptr)
            {
                if (_colliders[i]->getUserPointer() != nullptr)
                {
                    delete static_cast<btTransform*>(_colliders[i]->getUserPointer());
                }
                
                delete _colliders[i];
            }
        }

    }
    void addMultiBody(btMultiBody* _pMultiBody)
    {
        _multibody = _pMultiBody;
    }

    void updateTransforms()
    {
        rot_world_to_local.resize(0);
        local_origin_world_frame.resize(0);
        _multibody->forwardKinematics(rot_world_to_local,local_origin_world_frame);
        btTransform* ptr;
        btTransform tr, link_tr;
        btVector3 _offset;
        btVector3 _pos;
        btMultiBodyLinkCollider* _col;

        for (int i = 0; i < rot_world_to_local.size(); i++)
        {
            _col = _colliders[i];
            link_tr = _multibody->getLink(i).m_cachedWorldTransform;
            if (i == 0 )
            {
                if (_col != nullptr)
                    _col->setWorldTransform(link_tr);
                continue;
            }

            if (_col->getUserPointer() != nullptr)
            {
                ptr = static_cast<btTransform*>(_col->getUserPointer());
                _offset = ptr->getOrigin();
                // will multiply transforms tomorrow.

            }
        }
        
    }

    void printVector(btVector3& _v, std::string _str)
    {
        std::cout << _str << ": " << _v.x() <<  " " << _v.y() << " " << _v.z() << std::endl;
    }
    void printQuaternion(btQuaternion& _v, std::string _str)
    {
        if (_str.empty())
            _str = "Rot";
            std::cout << _str << ": " << _v.x() <<  " " << _v.y() << " " << _v.z() << " " << _v.w() << std::endl;
    }


    /* data */
    btMultiBody* _multibody;
    std::string _name;
    btAlignedObjectArray<btCollisionShape*> _colShapes;
    btAlignedObjectArray<btMultiBodyLinkCollider*> _colliders;
    std::unordered_map <std::string,int> _linkNameIndexMap;
    std::unordered_map <std::string,int> _jointNameIndexMap;
    std::unordered_map <int, btMultiBodyJointFeedback*> _jointFeedbackMap;
};

typedef mMultiBody RobotObject;

#endif
