#include <mObject.h>

namespace mviz
{
     // method description for mObject. starting ...
     mObject::mObject(std::string _name, int _type)
     {
          objectName = _name;
          type = _type;
     }


     const Eigen::Vector3d& mObject::getPosition()
     {
          return position;
     }

     const Eigen::Matrix3d & mObject::getRotation()
     {
          return rotation;
     }

     const Eigen::Vector3d& mObject::getScale()
     {
          return scale;
     }

     void mObject::setPosition(Ogre::Vector3 pos)
     {
          astd_Node->setPosition(pos);
     }

     void mObject::setPosition(Eigen::Vector3d &pos)
     {
          position = pos;
          astd_Node->setPosition(pos.x(), pos.y(), pos.z());
     }

     void mObject::setRotation(Eigen::Matrix3d  &rot)
     {
          rotation = rot;
          Eigen::Quaterniond Q(rot);
          Ogre::Quaternion q(Q.w(),Q.x(),Q.y(),Q.x());
          // astd_Node->rotate(q,Ogre::Node::TS_WORLD);            // Need to be examined.
          astd_Node->setOrientation(q);
          
     }

     void mObject::setRotation(double w, double x, double y, double z)
     {
          rotation = Eigen::Quaterniond(w,x,y,z).matrix();
          // Ogre::Quaternion q(w,x,y,z);
          // astd_Node->rotate(q, Ogre::Node::TS_WORLD);            // Need to be examined.
          // astd_Node->setOrientation(q);
          astd_Node->setOrientation(w,x,y,z);
     }

     void mObject::setRotation(Ogre::Quaternion qrot)
     {
          astd_Node->setOrientation(qrot);
     }

     void mObject::setScale(Ogre::Vector3 &scale)
     {
          astd_Node->setScale(scale);
     }

     void mObject::setEntityName(std::string& _entity_name)
     {
          entity_name = _entity_name;
     }

     void mObject::createEntity(Ogre::SceneManager* _scnMgr)
     {
          std::cout << "creating Entity from mObject : Name = " << objectName << std::endl;
          Ogre::Entity* _entity = _scnMgr->createEntity(entity_name);
          entityPtr = _entity;
          astd_Node->attachObject(_entity);
     }

     void mObject::setMeshFileName(std::string& _file_name)
     {
          mesh_file_name = _file_name;
     }

     void mObject::setSceneNode(Ogre::SceneNode* _node)
     {
          astd_Node = _node;
          astd_Node->setInheritOrientation(true);
     
     }

     void mObject::setMaterialColor(Ogre::ColourValue _color)
     {
          Ogre::MaterialPtr pmat = Ogre::MaterialManager::getSingleton().getByName("mat1");
          if (pmat == nullptr)
          {
               pmat = Ogre::MaterialManager::getSingleton().create("mat1","UserData");
          }
          pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
          pmat->getTechnique(0)->getPass(0)->setDiffuse(_color);
          // pmat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture();
          entityPtr->setMaterial(pmat);

     }

     void mObject::attachChildMesh(Ogre::SceneManager* _scM, std::string _meshName, Ogre::Vector3 pos, Ogre::Quaternion qrot)
     {
          Ogre::SceneNode* childNode = astd_Node->createChildSceneNode();
          Ogre::Entity* _childEntity = _scM->createEntity(_meshName);
          childNode->attachObject(_childEntity);
          
          childNode->setPosition(pos);
          childNode->setOrientation(qrot);
          // create a child struct and fill it.
          mChild* childPtr = new mChild;
          childPtr->childMeshName = _meshName;
          childPtr->childEntityName = _meshName;        // at this moment entity name = meshname;
          childPtr->_sNode = childNode;
          childPtr->rel_pos = pos;
          childPtr->rel_qrot = qrot;
          // store 
          children.push_back(childPtr);
     }
     // method description for mObject, ends here .

} // namespace mviz
