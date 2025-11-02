from math import pi
import marinholab.nottf2 as ntf2

def main():
    r1 = ntf2.rotx(phi=pi/2)
    r2 = ntf2.rotz(phi=pi/2)
    r12 = ntf2.quaternion_multiply(r1, r2)

    print(f"The quaternion multiplication of {r1} and {r2} is {r12}.")

if __name__ == '__main__':
    main()