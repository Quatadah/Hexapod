import pypot.dynamixel
import time
import math
from utils import *
import time

# from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os
from kinematics import * 
from getkey import getkey, keys


class P:
    def __init__(
        self,
        z=-0.06,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["1", "2", "3"]
        self.legs[2] = ["4", "5", "6"]
        self.legs[3] = ["7", "8", "9"]
        self.legs[4] = ["10", "11", "12"]
        self.legs[5] = ["13", "14", "15"]
        self.legs[6] = ["16", "17", "18"]


# import display

def setPositionToRobot(robot, params):
    
    for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            theta1 = 0
            theta2 = 0
            theta3 = 30
            if k == 2:
                theta1 = 45
            elif k == 3:
                theta1 = -45
            elif k == 5:
                theta1 = 45
            elif k == 6:
                theta1 = -45
            v[0].goal_position = theta1
            v[1].goal_position = theta2
            v[2].goal_position = theta3

# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]

def main():
    
    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

    robot = SimpleRobot(dxl_io)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)
    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)

    def movelegs():
        theta1 = 0
        for k in [1,2,3,4,5,6]:
            if k == 2:
                theta1 = 45
            elif k == 3:
                theta1 = -45
            elif k == 5:
                theta1 = 45
            elif k == 6:
                theta1 = -45
        # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            robot.legs[k][0].goal_position = math.degrees(targets[str(3*(k-1)+1)]+theta1)
            robot.legs[k][1].goal_position = math.degrees(targets[str(3*(k-1)+2)])
            robot.legs[k][2].goal_position = math.degrees(targets[str(3*(k-1)+3)])

        robot.tick_write(verbose=False)
        time.sleep(1/(params.freq))
    try:

        params = Parameters(
            freq=15,
            speed=1,
            z=-2.8,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        params1 = P()
        # TODO create this function instead:
        setPositionToRobot(robot, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
     
        #mode = "walk"
       
        while True:
            print("Possible modes : \n-Walk [w]\n-Rotate [r]\n-Dance [d]\n-Stop [s]")
            mode = input("Enter mode : ")
            if mode == 'w' or mode == 'W':
                mode = "walk"
            elif mode == 'r' or mode == 'R':
                mode = "rotate"
            elif mode == 'd' or mode == 'D':
                mode = "dance"
            elif mode == 's' or mode == 'S':
                break


            targets={}
            t = time.time()
            if mode == "rotate":
                freq = 2
                height = 0.07
                d_x = 0.16
                d_y = 0.05
                z = params.z
                

                tri1, tri2 = rotate(t, freq, d_x, d_y, height, z)

                for leg_id in [1,3,5]:
                    tetas = computeIK(tri1[0], tri1[1], tri1[2])
                    set_leg_angles(tetas, leg_id, targets, params1)

                for leg_id in [2,4,6]:
                    tetas = computeIK(tri2[0], tri2[1], tri2[2])
                    set_leg_angles(tetas, leg_id, targets, params1)

            
            if mode=="walk": 
                nord = 0
                sud = math.pi
                est = math.pi/2
                ouest = 3 * math.pi/2
                alpha = nord
                print("Possible directions : \n-North [up]\n-South [down]\n-East [right]\n-West [left]\-Break b")
                while True: 
                    key = getkey()
                    print(key)
                    if key == 'z':
                        alpha = nord
                    elif key == 's':
                        alpha = sud
                    elif key == 'd':
                        alpha = est
                    elif key == 'q':
                        alpha = ouest
                    elif key == 'b':
                        break
                    walk(1, params1, targets, alpha, 0.1, 0.07)
                    movelegs()
                    


            if mode =="dance":
                print("Press any key to rotate or b to quit")
                while True:
                    key = getkey()
                    if key == 'b':
                        break
                    else:
                        for leg_id in range(1, 7):
                            alphas = computeIKOriented(    
                            0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                            0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                            0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                            leg_id,
                            params1,
                            math.pi/3,
                            verbose=True,
                            )
                            set_leg_angles(alphas, leg_id, targets, params1)
                        movelegs()
            

            if mode == "rotate":
                print("Press any key to rotate or b to quit")
                while True:
                    key = getkey()
                    if key == 'b':
                        break
                    else:
                        t = time.time()
                        targets = {}
                        freq =2
                        height = 0.057
                        d_x = 0.16
                        d_y = 0.05
                        z = -0.1


                        tri1, tri2 = rotate(t, freq, d_x, d_y, height, z)

                        for leg_id in [1,3,5]:
                            tetas = computeIK(tri1[0], tri1[1], tri1[2])
                            set_leg_angles(tetas, leg_id, targets, params1)

                        for leg_id in [2,4,6]:
                            tetas = computeIK(tri2[0], tri2[1], tri2[2])
                            set_leg_angles(tetas, leg_id, targets, params1)
                
                        movelegs()

    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
