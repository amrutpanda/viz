#include <redisclient.h>

bool runloop = true;

void sighandler(int signum) {runloop = false;}

int main(int argc, char const *argv[])
{
    RedisClient redisclient("127.0.0.1",6379);
    redisclient.connect();
    std::string _key = "q1";
    Eigen::Vector3d v(3);
    while (runloop)
    {
        redisclient.getEigenMatrix(_key,v);
        std::cout << "v: " << v << std::endl;
    }
            
    return 0;
}
