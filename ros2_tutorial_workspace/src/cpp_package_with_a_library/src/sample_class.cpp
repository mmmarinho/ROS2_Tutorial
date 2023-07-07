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
 * @brief SampleClass::SampleClass the default constructor.
 */
SampleClass::SampleClass():
    a_private_qt_string_("I am a QString"),
    a_private_eigen3_matrix_((Eigen::Matrix2d() << 1,2,3,4).finished())
{

}

/**
 * @brief SampleClass::get_a_private_member.
 * @return an int with the value of a_private_member_.
 */
int SampleClass::get_a_private_member() const
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

/**
 * @brief SampleClass::to_string converts a SampleClass to a std::string representation.
 * @return a pretty(-ish) std::string representation of the object.
 */
std::string SampleClass::to_string() const
{
    std::stringstream ss;
    ss << "Sample_Class:: " << std::endl <<
          "a_private_member_ = "        << std::to_string(a_private_member_) << std::endl <<
          "a_private_qt_string_ = "     << a_private_qt_string_.toStdString() << std::endl <<
          "a_private_eigen3_matrix_ = " << a_private_eigen3_matrix_ << std::endl;
    return ss.str();
}

/**
 * @brief operator << the stream operator for SampleClass objects.
 * @param [in/out] the std::ostream to be modified.
 * @param [in] sc the SampleClass whose representation is to be streamed.
 * @return the modified os with the added SampleClass string representation.
 * @see SampleClass::to_string().
 */
std::ostream &operator<<(std::ostream &os, const SampleClass &sc)
{
    return os << sc.to_string();
}
