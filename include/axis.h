#include "mObject.h"

namespace mviz
{
    class  axis : public mObject
    {
    private:
        Ogre::SceneNode* x_node;
        Ogre::SceneNode* y_node;
        Ogre::SceneNode* z_node;

        void setAxisMaterial(Ogre::Entity* _ent, std::string matName);
    public:
        axis(mObject* _rObject);
        // void setScale(double _sx, double _sy, double _sz);
        ~ axis() {};
    };



} // namespace mviz

