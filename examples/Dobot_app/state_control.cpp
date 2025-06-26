#include <iostream>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <redisclient.h>
#include <teleop_redis_keys.h>

enum State {
    INIT = 0,
    FREE,
    SURGICAL,
    RECENTER
};
std::string state_names[] = {"INIT","FREE","SURGICAL","RECENTER"};

bool runloop = true;
void sig_handler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    signal(SIGINT,sig_handler);
    RedisClient redis_client;
    int val = 0;
    int robots = 2;

    int mode = 0;
    int robot_num = 0; 
    int prev_robot_num = robot_num;
    int _confirmer = 0;

    int group_num = 0;
    int state = 0;
    int prev_state = state;

    for (int i = 1; i <= robots; i++)
    {
        redis_client.set(createRobotRedisKey(STATE_TRANSITION_KEY,i),std::to_string(state));
        redis_client.set(createRobotRedisKey(ROBOT_NAME_KEY,i),std::to_string(i));

        redis_client.createIntGroupWriteCallback(i,createRobotRedisKey(STATE_TRANSITION_KEY,i),state,1);
        redis_client.createIntGroupWriteCallback(i,createRobotRedisKey(ROBOT_NAME_KEY,i),robot_num,1);

        redis_client.createIntGroupReadCallback(i,createRobotRedisKey(STATE_TRANSITION_KEY,i),state,1);
        redis_client.createIntGroupReadCallback(i,createRobotRedisKey(ROBOT_NAME_KEY,i),robot_num,1);
    }
        
    const char* device = "/dev/input/event3";

    int fd = open(device,O_RDONLY);
    if (fd == -1)
        throw std::runtime_error("Error while reading keyboard device file.\n Exiting...");

    input_event evt;
    std::cout << "Starting State controller.\n" << std::endl;

    while (runloop)
    {
        redis_client.executeAllReadCallbacks();
        bool _success = true;
        int count = 0;
        while (count < 3)
        {
            ssize_t bytes = read(fd,&evt,sizeof(evt));
            if (bytes != sizeof(evt) || evt.value != 1)
            {
                _success = false;
                break;
            }
            // if error in reading 
            if (!_success)
            {
                std::cout << "error while reading from keyboard. Read no. : " << count << std::endl;
                continue;;
            }
            count++;
            switch (count)
            {
                case 1:
                {
                    memcpy(&mode, &evt.code, sizeof(evt.code));
                    break;
                }
                case 2:
                {
                    if (mode == 41) {
                        memcpy(&robot_num, &evt.code, sizeof(evt.code));
                        robot_num -= 1;
                    }
                    else if(mode == 42)
                        memcpy(&state, &evt.code, sizeof(evt.code));
                    break;
                }
                case 3:
                {
                    if (evt.code == 28)
                        _confirmer = 1;
                    else
                        _confirmer = 0;
                    break;
                }
                default:
                    break;
            }
        }
        // if not success continue;
         
        // check whether state is valid.
        if (state > 4 || robot_num <1 || robot_num > robots)
        {
            std::cout << "Invalid state or robot_num. Press key from the beginning." << std::endl;
            continue;
        }


        if (robot_num != prev_robot_num)
        {
            state = INIT;
            std::cout << "Changing robot num from " << prev_robot_num << " to " << robot_num << std::endl;
            std::cout << "Setting new robot state to " << state_names[0] << std::endl;
        }
        
        if (state != prev_state)
        {
            std::cout << "Changing state from "<< state_names[prev_state] << " to " << state_names[state] << std::endl;
        }
        
        // print summary.
        std::cout << "mode: " << mode << ", state: " << state << std::endl; 
        // assign current state to previous state.
        if (_confirmer == 1)
        {
            prev_state = state;
            prev_robot_num = robot_num;
        }
        else
        {
            state = prev_state;
            prev_robot_num = robot_num;
        }
        
        _confirmer = 0;

        // ssize_t bytes = read(fd,&evt,sizeof(evt));
        // if (bytes != sizeof(evt) || evt.value != 1)
        //     continue;
        // memcpy(&val,&evt.code,sizeof(evt.code));
        // std::cout << "val: "<< val << std::endl;
        redis_client.executeGroupWriteCallbacks(robot_num);
    }
    
    // set state transition key to INIT.
    redis_client.set(STATE_TRANSITION_KEY,"0");
    std::cout << "closing state controller" << std::endl;

    return 0;
}
