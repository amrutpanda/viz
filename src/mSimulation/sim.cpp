#include <iostream>
#include "Ogre.h"


// #include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h"

int main(int argc, char const *argv[])
{
    std::cout << "Hello " << std::endl;
    b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();
    bool isConnected = sim->connect(eCONNECT_DIRECT);

    
        
    
    return 0;
}
