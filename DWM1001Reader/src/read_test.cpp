#include "ros/ros.h"
#include <fcntl.h>

int main(int argc, char **argv)
{
    int n = 0;
    char buf;

    std::cout << "iniciado";

    int USB = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );

    std::cout << "aberto";

    while ( USB )
    {
        //std::cout << "while";
        n = read( USB, &buf, 1 );
        //std::cout << n;
        std::cout << buf;
        std::cout << '\n';
    }
    std::cout << "DISCONECTED\n";

}