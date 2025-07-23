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

int num_robots = 2;
int num_states = 4;

bool runloop = true;
void sig_handler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    signal(SIGINT,sig_handler);
    RedisClient redis_client;

    int robot_num = 0;
    int prev_robot_num = robot_num;
    int state = INIT;
    int prev_state = state;
    std::string r,s;

    // initialisation of the key to redis i.e. state as 0 and robot num as 1.
    redis_client.set(STATE_TRANSITION_KEY,"0");
    redis_client.set(ROBOT_NAME_KEY,"1");

    std::cout << "Starting state controller....." << std::endl;

    while (runloop)
    {
        robot_num = std::stoi(redis_client.get(ROBOT_NAME_KEY));
        state = std::stoi(redis_client.get(STATE_TRANSITION_KEY));
        std::cout << "-------------------------------------------------" << std::endl;
        for (int i = 0; i < 2; i++)
        {
            if ( i == 0)
            {
                std::cout << "Enter robot no.: " << std::endl;
                std::cin >> r;
            }
            else if (i == 1)
            {
                std::cout << "Enter State : " << std::endl;
                std::cin >> s;
            } 
            // clear the input buffer.
            std::cin.clear();
        }
        // convert r and s to integers.
        try
        {
            robot_num = std::stoi(r);
            state = std::stoi(s);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            std::cout << "Invalid input. Enter again" << std::endl;
            continue;
        }
        

        // check input validity
        if (robot_num < 1 || robot_num > num_robots || state < 0 || state > num_states)
        {
            std::cout << "Invalid input. Enter again." << std::endl;
            continue;
        }
        else
        {
            if (robot_num != prev_robot_num)
            {
                state = INIT;
                std::cout << "Change in robot number. from " << prev_robot_num << " to "
                                            << robot_num << std::endl;
                std::cout << "changing state " << state_names[prev_state] << " to "
                                            << state_names[0] << std::endl;
            }
            
        }
        std::cout << "robot number: " << robot_num << " ; state: " << state_names[state] << std::endl; 
        std::cout << "------------------------------------------------------------------" << std::endl;
        prev_state = state;
        prev_robot_num = robot_num;

        // write to redis.
        std::cout << "Writing to redis" << std::endl;
        redis_client.set(ROBOT_NAME_KEY,std::to_string(robot_num));
        redis_client.set(STATE_TRANSITION_KEY,std::to_string(state));
    }
    
    // set state transition key to INIT.
    redis_client.set(STATE_TRANSITION_KEY,"0");
    redis_client.set(ROBOT_NAME_KEY,"1");
    std::cout << "closing state controller" << std::endl;

    return 0;
}
