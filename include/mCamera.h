#pragma once
#include <iostream>
#include <Ogre.h>
#include <OgreCamera.h>
#include <OgreInput.h>

namespace mviz
{
    class mCamera : public OgreBites::InputListener
    /*
        Only orbit style supported at this moment. Will add to other supports
    */
    {
    private:
        Ogre::SceneNode* camFixedAxisNode;
        Ogre::SceneNode* camNode;
        Ogre::SceneNode* lightNode;
        Ogre::SceneNode* targetNode;
        Ogre::Vector3 mTarget;
        float dist;
    public:
        mCamera(Ogre::SceneNode* _cameraNode);
        ~mCamera();
        void setMouseSensitivity(float& value);
        bool mousePressed(const OgreBites::MouseButtonEvent& evt);
        bool mouseReleased(const OgreBites::MouseButtonEvent& evt);
        bool mouseMoved(const OgreBites::MouseMotionEvent& evt);
        bool mouseWheelRolled(const OgreBites::MouseWheelEvent& evt);
    protected:
        bool mOrbiting;
        bool mMoving;
        float sensitivity = 0.01;
        float _getdistFromTarget();
    };
} // namespace mviz


