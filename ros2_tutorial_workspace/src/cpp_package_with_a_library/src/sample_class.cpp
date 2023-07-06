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
#include <cpp_package_with_a_library/sample_class.hpp>

/**
 * @brief SampleClass::get_a_private_member.
 * @return an int with the value of a_private_member_.
 */
int SampleClass::get_a_private_member()
{
    return a_private_member_;
}

/**
 * @brief SampleClass::set_a_private_member.
 * @param value The new value for a_private_member_.
 */
void SampleClass::set_a_private_member(int value)
{
    a_private_member_ = value;
}

/**
 * @brief SampleClass::sum_of_squares.
 * @param a The first number.
 * @param b The second number.
 * @return a*a + 2*a*b + b*b.
 */
double SampleClass::sum_of_squares(const double &a, const double &b)
{
    return a*a + 2*a*b + b*b;
}
