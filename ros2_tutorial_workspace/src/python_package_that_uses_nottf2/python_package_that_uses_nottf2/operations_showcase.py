"""
MIT LICENSE

Copyright (C) 2023-25 Murilo Marques Marinho (www.murilomarinho.info)

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
"""
from math import pi
from geometry_msgs.msg import Quaternion
import marinholab.nottf2 as ntf2

def main():
    phi: float = pi

    r1: Quaternion = ntf2.rx(phi=phi)
    print(f"The rotation of {phi} radians about the x-axis is r1={r1}.")

    r1_conj : Quaternion = ntf2.rotation_inverse(r1)
    print(f"The inverse rotation of r1 is r1_conj={r1_conj}.")

    r2: Quaternion = ntf2.rz(phi=phi)
    print(f"The rotation of {phi} radians about the z-axis is r2={r2}.")

    r12: Quaternion = ntf2.quaternion_multiply(r1, r2)
    print(f"The quaternion multiplication of r12=r1r2 is r12={r12}.")

if __name__ == '__main__':
    main()