/*
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "print_forever_node.hpp"

/**
 * @brief PrintForeverNode::PrintForeverNode Default constructor.
 */
PrintForeverNode::PrintForeverNode():
    rclcpp::Node("print_forever_cpp"),
    timer_period_(0.5),
    print_count_(0)
{
    //(Smart) pointers at the one thing that it doesn't matter much if they are not initialized in the member initializer list
    //and this is a bit more readable.
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(timer_period_*1e3)),
                std::bind(&PrintForeverNode::_timer_callback, this) //Note here the use of std::bind to build a single argument
                );
}

/**
 * @brief PrintForeverNode::_timer_callback periodically prints class info using RCLCPP_INFO.
 */
void PrintForeverNode::_timer_callback()
{
    RCLCPP_INFO_STREAM(get_logger(),
                       std::string("Printed ") +
                       std::to_string(print_count_) +
                       std::string(" times.")
                       );
    print_count_++;
}
