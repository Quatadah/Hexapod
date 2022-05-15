import math
from constants import *
from scipy.optimize import minimize
import numpy as np
import interpolation
import time


use_mm=USE_MM_OUTPUT
l1=constL1
l2=constL2
l3=constL3
sign = -1
# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def elkashi(a, b, c, sign=-1):
    if a * b == 0:
        print("error")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    use_rads=USE_RADS_INPUT,
):
    
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]




# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    verbose=False,
    use_rads=USE_RADS_OUTPUT
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = elkashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + elkashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [ THETA1_MOTOR_SIGN * theta1,
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction),
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction)]

    else:
        result = [ THETA1_MOTOR_SIGN * math.degrees(theta1),
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction)  ]
    return result


#return the vector (x,y,z) with a teta rotation in the x,y plan
def rotaton_2D(x, y, z, teta):
    vect = np.dot(np.array([[np.cos(teta), -np.sin(teta)],
                            [np.sin(teta), np.cos(teta)]]), np.array([x, y]))
    return [vect[0], vect[1], z]



# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# and an angle teta that chage the direction of the movement
# returns the angles to apply to the 3 axes
def computeIKOriented(
    x,
    y,
    z,
    leg_id,
    params,
    teta,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
):


    [x,y,z] = rotaton_2D(x, y, z,params.legAngles[leg_id-1] + teta)
    x += params.initLeg[leg_id-1][0]
    y += params.initLeg[leg_id-1][1]
    z += params.z

    return computeIK(
        x,
        y,
        z,
        verbose,
        use_rads
        )


def set_leg_angles_2(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]

def walk(freq, params, targets, teta, length, height):
    t = time.time()
    if freq == 0:
        tri1 = [0,0,0]
        tri2 = [0,0,0]
    else:
        T = 1 / freq
        triangle = interpolation.LinearSpline3D()
        triangle.add_entry(0, length / 2, 0,  params.z)
        triangle.add_entry(T / 2, -length / 2, 0, params.z)
        triangle.add_entry(3 * T / 4, 0, 0, height)
        triangle.add_entry(T, length / 2, 0,  params.z)
        tri1 = triangle.interpolate(t % T)
        tri2 = triangle.interpolate((t + T/2) % T)
    for leg_id in [1,3,5]:
        alphas = computeIKOriented(tri1[0], tri1[1], tri1[2], leg_id, params, teta, verbose=True)
        set_leg_angles_2(alphas, leg_id, targets, params)
    for leg_id in [2,4,6]:
        alphas = computeIKOriented(tri2[0], tri2[1], tri2[2], leg_id, params, teta, verbose=True)
        set_leg_angles_2(alphas, leg_id, targets, params)





































































"""
def getPosition():
    return (round(position[0], 1), round(position[1], 1))



def toupie(t, freq, d_x, d_y, height, z) :
    return """