#include <dmObject.h>

namespace mviz
{
    dmObject::dmObject()
    {
        
    }

    void dmObject::createManualObject(std::string _objName)
    {
        std::cout << "Hello I am here" << std::endl;
        man = mScnMgr->createManualObject(_objName);
        man->setDynamic(true);

        // material scripts contains ambient value as vertexcolour so that it follows the color set
        // in manual object color method. The point size is set accordingly. It can be changed as per the need.
        // ck the material script inside resource folder for more clarity.

        mat = Ogre::MaterialManager::getSingleton().getByName("FlatVertexColour","UserData");  
        if (mat.get() == nullptr)
            std::cout << "could not find the material: " << "FlatVertexColour" << std::endl;
        man->begin(mat,Ogre::RenderOperation::OT_POINT_LIST);
        std::cout << mat->getName() << std::endl;
        
        for (int i = 0; i < 1000; i++)
        {
            man->position(10, 10 , i);
            man->colour(0.1, 0.1, 0.01*i);
            
        }
        man->end();
        
        getSceneNode()->attachObject(man);
        std::cout << "manual creation over." << std::endl;
    }
    void dmObject::updateBuffer()
    {
        // TO-DO: It will be updated.
    }
} // namespace mviz
