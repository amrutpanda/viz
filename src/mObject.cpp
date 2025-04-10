#include <mObject.h>

namespace mviz
{
     // method description for mObject. starting ...
     mObject::mObject(std::string _name, int _type)
     {
          objectName = _name;
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
         dummy_Node->setPosition(pos);
     }

     void mObject::setPosition(Eigen::Vector3d &pos)
     {
          position = pos;
          dummy_Node->setPosition(pos.x(), pos.y(), pos.z());
          // astd_Node->setPosition(pos.x(), pos.y(), pos.z());
          // astd_Node->setInheritOrientation()
     }

     void mObject::setPositionLocal(Ogre::Vector3 pos)
     {
          astd_Node->setPosition(pos);
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
         
          dummy_Node->setOrientation(w,x,y,z);
     }

     void mObject::setRotation(Ogre::Quaternion qrot)
     {
          dummy_Node->setOrientation(qrot);
     }

     void mObject::setRotationLocal(Ogre::Quaternion qrot)
     {
          astd_Node->setOrientation(qrot);
     }

     void mObject::setRotationLocal(double w, double x, double y, double z)
     {
          astd_Node->setOrientation(w,x,y,z);
     }

     void mObject::setScale(Ogre::Vector3 &scale)
     {
          astd_Node->setScale(scale);
     }

     void mObject::setScale(double _sx, double _sy, double _sz)
     {
          astd_Node->setScale(_sx, _sy, _sz);
     }

     void mObject::setVisible(bool _flag)
     {
          std::cout << "turning visibility of mObject, Name " << objectName << " to " << _flag << std::endl;
          astd_Node->setVisible(_flag);
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

     // void mObject::setMeshFileName(std::string& _file_name)
     // {
     //      mesh_file_name = _file_name;
     // }

     void mObject::setSceneNode(Ogre::SceneNode* _node)
     {
          //test.
          dummy_Node = _node;
          astd_Node = dummy_Node->createChildSceneNode();

          astd_Node->setPosition(Ogre::Vector3(0));
          astd_Node->setOrientation(Ogre::Quaternion(1,0,0,0));
          //test
          // astd_Node = _node;
          astd_Node->setInheritOrientation(true);

          // set sceneManager.
          mScnMgr = _node->getCreator();
     
     }

     
     void mObject::setMaterialColor(Ogre::ColourValue _color)
     {
          Ogre::MaterialPtr pmat = Ogre::MaterialManager::getSingleton().getByName("Default_mat");
          Ogre::MaterialPtr cmat;
          if (pmat == nullptr)
          {
               cmat = Ogre::MaterialManager::getSingleton().create("Default_mat","UserData");
          }
          else
          {
               cmat = pmat->clone(objectName);
          }
          
          cmat->getTechnique(0)->getPass(0)->setAmbient(_color);
          cmat->getTechnique(0)->getPass(0)->setDiffuse(_color);
          // pmat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture();
          entityPtr->setMaterial(cmat);

     }

     

     void mObject::attachChildMesh(Ogre::SceneManager* _scM, std::string _meshName, Ogre::Vector3 pos,
                                    Ogre::Quaternion qrot, Ogre::Vector3 scale)
     {
          Ogre::SceneNode* childNode = astd_Node->createChildSceneNode();
          Ogre::Entity* _childEntity = _scM->createEntity(_meshName);
          // _childEntity->setCastShadows(true);
          
          childNode->attachObject(_childEntity);
          childNode->setScale(scale);
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

     Ogre::SceneNode* mObject::getChildMeshNode(std::string _chMshName)
     {
          mChild* ch;
          for (int i = 0; i < children.size(); i++)
          {
               ch = children[i];
               if (ch->IsNameMatching(_chMshName))
               {
                    return ch->_sNode;
               }
               
          }
          throw std::runtime_error("Child Mesh doesn't exist. Mesh name: " + _chMshName + " mObject name: " + objectName);
          
     }

     mObject* mObject::getChildObject(std::string _chObjName)
     {
         try
         {
               return child_objects[_chObjName];
         }
         catch(const std::exception& e)
         {
               return nullptr;
         }
          
     }

     bool mObject::IsChildMeshExists(std::string& _chName)
     {
          mChild* ch;
          for (int i = 0; i < children.size(); i++)
          {
               ch = children[i];
               if (ch->IsNameMatching(_chName))
               {
                    return true;
               }
               
          }
          return false;
          
     }
     void mObject::attachNode(Ogre::SceneNode* _sNode, Ogre::Vector3 rel_pos, Ogre::Quaternion rel_qrot)
     {
          Ogre::SceneNode* _pNode = _sNode->getParentSceneNode();
          if (_pNode == nullptr)
          {
               std::cout << "The node does have a parent node. Detaching it and attaching it." << std::endl;
               _pNode->removeChild(_sNode);
          }
          
          astd_Node->addChild(_sNode);
          _sNode->setPosition(rel_pos);
          _sNode->setOrientation(rel_qrot);
     }

     void mObject::attachObject(mObject* _Object)
     {
          // TO-DO: detaching the _Object from its parent node and include it in if statement.
          if (child_objects[_Object->objectName] == nullptr)
          {
               child_objects[_Object->objectName] = _Object;
               _Object->setSceneNode(astd_Node->createChildSceneNode());
          }
          else
               throw std::runtime_error("Cannot attach mObject: " + _Object->objectName + " to " 
                                        + objectName + "as it already exists");
          // _Object->getSceneNode()->getParentSceneNode()->removeChild(_Object->getSceneNode());
          // astd_Node->addChild(_Object->getSceneNode());
                  
     }

     bool mObject::IsChildObjectExists(std::string _chObjName)
     {

          try
          {
               child_objects.at(_chObjName);
               return true;

          }
          catch(const std::exception& e)
          {
               // std::cerr << e.what() << '\n';
               return false;
          }
          
          
     }

     void mObject::setChildMeshVisible(std::string& _chName, bool _flag)
     {
          mChild* ch;
          for (int i = 0; i < children.size(); i++)
          {
               ch = children[i];
               if (ch->IsNameMatching(_chName))
               {
                    ch->setVisible(_flag);
                    return;
               }
               
          }
          throw std::runtime_error("No Child Mesh present with name: " + _chName + " in mObject: " + objectName);
          
     }
     void mObject::setChildObjectVisible(std::string& _chObjName, bool _flag)
     {
          mObject* chObj = child_objects.at(_chObjName);
          if (chObj != nullptr)
          {
               chObj->getSceneNode()->setVisible(_flag,true);
          }
          else
          {
               throw std::runtime_error("No Child mObject present with name: " + _chObjName + " in mObject: " + objectName);
          }     
          
     }

     void mObject::setAxis()
     {
          if (astd_Node == nullptr)
               std::runtime_error("mObject Not part of scenegraph. Run setSceneNode command to proceed. Name: " + objectName);
          if (objectName != "AXIS" && !IsChildObjectExists("AXIS"))
               axis* ax =  new axis(this);
          else
               std::cout << "This object is an 'axis' mObject. Cannot set axis. Ignoring command." << std::endl;
     }

     void mObject::setAxisVisible(bool _flag)
     {
          // std::cout << "setting axis visible. Name: " << objectName << std::endl;
          std::string axis_object_name = "AXIS";
          if (objectName != axis_object_name || IsChildObjectExists(axis_object_name))
               setChildObjectVisible(axis_object_name,_flag);
          else
               std::cout << "This is an 'axis' mObject. Ignoring setvisible command." << std::endl;
     }

     void mObject::destroymObject(bool _destroy_ogreNode)
     {
          mObject* ax;
          if (IsChildObjectExists("AXIS"))
          {
               ax = getChildObject("AXIS");
               ax->destroymObject(true);
          }

          // delete the attached child meshes.
          for (int i = 0; i < children.size(); i++)
          {
               children[i]->_sNode->destroyAllObjects();
               astd_Node->removeAndDestroyChild(children[i]->_sNode);
          }
          // delete the main ogrenode and dummy ogrenode.
          if (_destroy_ogreNode)
          {
               dummy_Node->removeAndDestroyChild(astd_Node);
               dummy_Node->getParentSceneNode()->removeAndDestroyChild(dummy_Node);
          }
          // enable the flag.
          _isNodesDestroyed = true;
     }

     mObject::~mObject()
     {
         if (! _isNodesDestroyed)
         {
               // destroymObject();
         }
         
     }
     // method description for mObject, ends here.

} // namespace mviz
