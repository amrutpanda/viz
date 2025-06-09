/**
 * Haptic controller Class to control haptic device.
 * This class works by setting up a redis connection with the chai_haptic_device_redis_driver.
*/

#ifndef _HAPTIC_CONTROLLER_H
#define _HAPTIC_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>

namespace Primitives
{
    
    class HapticController
    {
    private:
        
    public:
        HapticController(/* args */);
        ~HapticController();
    };


} // namespace Primitives



#endif