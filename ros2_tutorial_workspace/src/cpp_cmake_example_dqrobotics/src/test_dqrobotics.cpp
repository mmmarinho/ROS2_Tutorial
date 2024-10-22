#include <iostream>
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;

int main()
{
    auto x = 2*i_ + 3*j_;
    auto y = 3*i_ - j_;
    std::cout<<x-y<<std::endl;
    return 0;
}