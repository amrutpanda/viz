#include <mCamera.h>

namespace mviz
{
    mCamera::mCamera(Ogre::SceneNode* _cameraNode)
    {
        mOrbiting = false;
        mMoving = false;
        mTarget = Ogre::Vector3(0);
        // setup camera node.
        Ogre::SceneManager* _scnMgr = _cameraNode->getCreator();
        targetNode = _scnMgr->getRootSceneNode()->createChildSceneNode();
        // detach the camera node from RootsceneNode and attach to camFixedAxisNode;
        Ogre::SceneNode* parentNode = _cameraNode->getParentSceneNode();
        parentNode->removeChild(_cameraNode);
        targetNode->createChildSceneNode()->addChild(_cameraNode);

        camNode = _cameraNode;
        camNode->lookAt(targetNode->getPosition(),Ogre::Node::TS_PARENT);
        // create a light that will move along with the camera.
        Ogre::Light* light = _scnMgr->createLight("CameraLight");
        Ogre::SceneNode* lightNode = camNode->createChildSceneNode();
        lightNode->attachObject(light);
        
    }

    mCamera::~mCamera()
    {

    }

    float mCamera::_getdistFromTarget()
    {
        Ogre::Vector3 cpos = camNode->getPosition();
        return (mTarget - cpos).length();
        
    }

    bool mCamera::mousePressed(const OgreBites::MouseButtonEvent& evt)
    {
        if (evt.button == OgreBites::BUTTON_LEFT)
        {
            mOrbiting = true;
        }
        else if (evt.button == OgreBites::BUTTON_RIGHT)
        {
            mMoving = true;
        }
        
        return InputListener::mousePressed(evt) ;
    }

    bool mCamera::mouseReleased(const OgreBites::MouseButtonEvent& evt)
    {
        if (evt.button == OgreBites::BUTTON_LEFT)
        {
            mOrbiting = false;
        }
        else if (evt.button == OgreBites::BUTTON_RIGHT)
        {
            mMoving = false;
        }
        return InputListener::mouseReleased(evt);
    }

    bool mCamera::mouseMoved(const OgreBites::MouseMotionEvent& evt)
    {
        if (mOrbiting)
        {
            targetNode->pitch(Ogre::Radian(-evt.yrel*sensitivity),Ogre::Node::TS_PARENT);
            targetNode->yaw(Ogre::Radian(-evt.xrel*sensitivity),Ogre::Node::TS_PARENT);
            // camNode->lookAt(targetNode->getPosition(),Ogre::Node::TS_PARENT);
            
        }
        else if(mMoving)
        {
            // std::cout << "Mamuni mishra here!!!!!!!!!" << std::endl;
            targetNode->translate(Ogre::Vector3(-evt.xrel*sensitivity*1, evt.yrel*sensitivity*1,0));
        }
        
        // return InputListener::mouseMoved(evt);
        return true;
    }

    bool mCamera::mouseWheelRolled(const OgreBites::MouseWheelEvent& evt)
    {
        if (evt.y != 0)
        {
            Ogre::Vector3 dir_vector = camNode->getPosition();
            // dir_vector.normalise();
            camNode->translate(-dir_vector* evt.y * 0.1);
        }
        return InputListener::mouseWheelRolled(evt);
    }
    
} // namespace mviz
