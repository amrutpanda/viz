#include <mObject.h>

namespace mviz
{
     // method description for mObject. starting ...
     mObject::mObject(std::string _name, int _type)
     {
          objectName = _name;
          type = _type;
     }

     // mObject::~mObject()
     // {
     //      if (mesh_assigned)
     //      {
     //           delete mesh;
     //      }
     // }

     void mObject::assign_mesh(Ogre::Mesh* m)
     {
          mesh = m;
          mesh_assigned = true;
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
          Ogre::Quaternion q(w,x,y,z);
          // astd_Node->rotate(q, Ogre::Node::TS_WORLD);            // Need to be examined.
          astd_Node->setOrientation(q);
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
          if (pmat.isNull())
          {
               pmat = Ogre::MaterialManager::getSingleton().create("mat1","UserData");
          }
          pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
          pmat->getTechnique(0)->getPass(0)->setDiffuse(_color);
          // pmat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture();
          entityPtr->setMaterial(pmat);

     }
     // method description for mObject, ends here .

     void convert_urdf_to_mvector(mVector& v_m, urdf::Vector3 v_u)
     {
          v_m << v_u.x, v_u.y, v_u.z ;
     }

} // namespace mviz
