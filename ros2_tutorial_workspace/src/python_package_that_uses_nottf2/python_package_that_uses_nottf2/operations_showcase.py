from math import pi
from geometry_msgs.msg import Quaternion
import marinholab.nottf2 as ntf2

def main():
    phi: float = pi / 2.0

    r1: Quaternion = ntf2.rotx(phi=phi)

    print(f"The rotation of {phi} radians about the x-axis is {r1}.")

    r2: Quaternion = ntf2.rotz(phi=phi)

    print(f"The rotation of {phi} radians about the z-axis is {r1}.")

    r12: Quaternion = ntf2.quaternion_multiply(r1, r2)

    print(f"The quaternion multiplication of r12=r1r2 is {r12}.")

if __name__ == '__main__':
    main()