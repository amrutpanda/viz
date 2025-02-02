#include "mObject.h"

namespace mviz
{
    axis::axis(mObject* _rObject)
    {
        objectName = "AXIS";
        _rObject->attachObject(this);

        Ogre::SceneNode* mainNode = getSceneNode();
        x_node = mainNode->createChildSceneNode();
        y_node = mainNode->createChildSceneNode();
        z_node = mainNode->createChildSceneNode();

        Ogre::SceneManager* _scnMgr = mainNode->getCreator();

        if (_scnMgr == nullptr)
            throw std::runtime_error("SceneManager cannot be found while creating axis components. ObjectName" + objectName );

        Ogre::Entity* x_entity = _scnMgr->createEntity("mCylinder.mesh");
        setAxisMaterial(x_entity,"RED");
        Ogre::Entity* y_entity = _scnMgr->createEntity("mCylinder.mesh"); 
        setAxisMaterial(y_entity,"GREEN");
        Ogre::Entity* z_entity = _scnMgr->createEntity("mCylinder.mesh");
        setAxisMaterial(y_entity,"BLUE");

        x_node->attachObject(x_entity);
        y_node->attachObject(y_entity);
        z_node->attachObject(z_entity);
    
        x_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_Y));
        x_node->setPosition(Ogre::Vector3(5,0,0));

        y_node->setOrientation(Ogre::Quaternion(Ogre::Degree(-90),Ogre::Vector3::UNIT_X));
        y_node->setPosition(Ogre::Vector3(0,5,0));

        z_node->setOrientation(Ogre::Quaternion(Ogre::Degree(0),Ogre::Vector3::UNIT_Z));
        z_node->setPosition(Ogre::Vector3(0,0,5));

        setScale(0.01, 0.01, 0.01);
    }
    
    void axis::setAxisMaterial(Ogre::Entity* _ent, std::string matName = "")
    {
        // Ogre::ColourValue _color;
        // if (matName == "RED")
        //     _color = Ogre::ColourValue(1,0, 0.0, 0.0);
        // else if(matName == "GREEN")
        //     _color = Ogre::ColourValue(0,0, 1.0, 0.0);
        // else if(matName == "BLUE")
        //     _color = Ogre::ColourValue(0,0, 0.0, 1.0);
        // else
        //     _color = Ogre::ColourValue(1,0, 1.0, 1.0);
        std::cout << "checking for selected material in Ogre scenegraph\n";
        // check whether the material exists otherwise create new material.
        Ogre::MaterialPtr pmat = Ogre::MaterialManager::getSingleton().getByName(matName,"UserData");
        if ( pmat == nullptr)
        {
            std::cout << "Creating new material. " << matName << std::endl;
             Ogre::ColourValue _color;

            if (matName == "RED")
                _color = Ogre::ColourValue(1,0, 0.0, 0.0);
            else if(matName == "GREEN")
                _color = Ogre::ColourValue(0.0, 1.0, 0.0);
            else if(matName == "BLUE")
                _color = Ogre::ColourValue(0,0, 0.0, 0.9);
            // else
                // _color = Ogre::ColourValue(1,0, 1.0, 1.0);

            pmat = Ogre::MaterialManager::getSingleton().create(matName,"UserData");

            pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
            // pmat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(1,1,1));
            pmat->getTechnique(0)->getPass(0)->setDiffuse(_color);
            // pmat->getTechnique(0)->getPass(0)->setSpecular(_color);
        }

        _ent->setMaterial(pmat);


    }
} // namespace mviz
