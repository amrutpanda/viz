#include <mObject.h>

namespace mviz
{
     // method description for mObject. starting ...
     mObject::mObject(std::string name, int type)
     {
          name = Name;
          type = type;
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

     const mVector& mObject::getPosition()
     {
          return position;
     }

     const mMatrix& mObject::getRotation()
     {
          return rotation;
     }

     const mVector& mObject::getScale()
     {
          return scale;
     }

     void mObject::setPosition(mVector &pos)
     {
          position = pos;

     }

     void mObject::setRotation(mMatrix &rot)
     {
          rotation = rot;
     }

     void mObject::setScale(Ogre::Vector3 &scale)
     {
          astd_Node->setScale(scale);
     }

     // method description for mObject, ends here .

     void convert_urdf_to_mvector(mVector& v_m, urdf::Vector3 v_u)
     {
          v_m << v_u.x, v_u.y, v_u.z ;
     }

} // namespace mviz
