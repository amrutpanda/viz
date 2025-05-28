#include <iostream>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <redis_keys.h>
#include <redisclient.h>

bool runloop = true;
void sig_handler(int signum) {runloop = false;}
RedisClient redis_client;
double _dx = 0.01;
double _alpha_z = 0;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sig_handler);

    Eigen::Vector3d x_c, x_t;
    // TODO: Need to implement rotation of coordinate frame.
    redis_client.connect();
    redis_client.createEigenReadCallback(CURRENT_EE_POSE,x_c);
    redis_client.createEigenWriteCallback(TARGET_EE_POSE,x_t);

    const char* device = "/dev/input/event3";

    int fd = open(device,O_RDONLY);
    if (fd == -1)
        throw std::runtime_error("Error while reading keyboard device file.\n Exiting...");

    input_event evt;
    std::cout << "Starting to read from Keyboard\n" << std::endl;

    while (runloop)
    {
        redis_client.executeAllWriteCallbacks();
        ssize_t bytes = read(fd,&evt,sizeof(evt));
        if (bytes != sizeof(evt))
            continue;
        switch (evt.code)
        {
        case 30:
            {
                std::cout << "A" << std::endl;
                x_t = x_c + Eigen::Vector3d(_dx,0,0);
                break;
            }
        case 31:
            {
                std::cout << "S" << std::endl;
                x_t = x_c + Eigen::Vector3d(0,-_dx,0);
                break;
            }
        case 17:
            {
                std::cout << "W" << std::endl;
                x_t = x_c + Eigen::Vector3d(0,_dx,0);
                break;
            }
        case 32:
            {
                std::cout << "D" << std::endl;
                x_t = x_c + Eigen::Vector3d(-_dx,0,0);
                break;
            }
        case 103:
            {
                std::cout << "up" << std::endl;
                x_t = x_c + Eigen::Vector3d(0,0,_dx);
                break;
            }
        case 108:
            {
                std::cout << "down" << std::endl;
                x_t = x_c + Eigen::Vector3d(0,0,-_dx);
                break;
            }
        default:
            break;
        }
        // std::cout << "Key pressed: " << evt.code << std::endl;
        redis_client.executeAllWriteCallbacks();
    }

    close(fd);
    return 0;
}
