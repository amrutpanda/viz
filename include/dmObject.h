#pragma once

#include <mObject.h>

namespace mviz
{    
    class dmObject : public mObject
    {
    private:
        std::string mesh_name;
        Ogre::ManualObject* man;
        Ogre::MaterialPtr mat;
    public:
        dmObject(/* args */);
        void createManualObject(std::string _objName);
        void updateBuffer();
        ~dmObject(){};
    };
} // namespace mviz


