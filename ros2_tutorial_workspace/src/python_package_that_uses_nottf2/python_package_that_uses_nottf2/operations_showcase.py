from math import pi
from geometry_msgs.msg import Quaternion
import marinholab.nottf2 as ntf2

def main():
    phi: float = pi

    r1: Quaternion = ntf2.rotx(phi=phi)
    print(f"The rotation of {phi} radians about the x-axis is r1={r1}.")

    r1_conj : Quaternion = ntf2.rotation_inverse(r1)
    print(f"The inverse rotation of r1 is r1_conj={r1_conj}.")

    r2: Quaternion = ntf2.rotz(phi=phi)
    print(f"The rotation of {phi} radians about the z-axis is r2={r2}.")

    r12: Quaternion = ntf2.quaternion_multiply(r1, r2)
    print(f"The quaternion multiplication of r12=r1r2 is r12={r12}.")

if __name__ == '__main__':
    main()