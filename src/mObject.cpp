#include <mObject.h>

namespace mviz
{
     // method description for mObject. starting ...
     mObject::mObject(std::string _name, int _type)
     {
          objectName = _name;
          type = _type;
     }

     mObject::~mObject()
     {
          if (mesh_assigned)
          {
               delete mesh;
          }
     }

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

     }

     void mObject::setRotation(Eigen::Matrix3d  &rot)
     {
          rotation = rot;
     }

     void mObject::setScale(Ogre::Vector3 &scale)
     {
          astd_Node->setScale(scale);
     }

     void mObject::setEntityName(std::string& _entity_name)
     {
          entity_name = _entity_name;
     }

     void mObject::setMeshFileName(std::string& _file_name)
     {
          mesh_file_name = _file_name;
     }

     void mObject::setSceneNode(Ogre::SceneNode* _node)
     {
          astd_Node = _node;
     }
     // method description for mObject, ends here .

     void convert_urdf_to_mvector(mVector& v_m, urdf::Vector3 v_u)
     {
          v_m << v_u.x, v_u.y, v_u.z ;
     }

} // namespace mviz
