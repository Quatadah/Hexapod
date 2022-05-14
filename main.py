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
        self.legs[6] = ["4", "5", "6"]
        self.legs[5] = ["7", "8", "9"]
        self.legs[2] = ["10", "11", "12"]
        self.legs[3] = ["13", "14", "15"]
        self.legs[4] = ["16", "17", "18"]


# import display

def setPositionToRobot(theta1, theta2, theta3, robot, params):
    for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            v[0].goal_position = theta1
            v[1].goal_position = theta2
            v[2].goal_position = theta3

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
    try:

        params = Parameters(
            freq=50,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        params1 = P()
        # TODO create this function instead:
        setPositionToRobot(10, 0, 0, robot, params)
        while True:
            targets={}
            print("avant  robot.legs.items():\n ",  robot.legs.items())
            print("****************************************************")
            walk(params.freq, params1, targets, math.pi/4, 0.1, 0.1)
            for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
                print("avant affectation v est \n", len(v) )
                print("le v[i] est \n", v[0].goal_position)
             
            i = 1
            for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
                print("targets est \n", targets)
                v[0].goal_position = targets[str(i)]
                v[1].goal_position = targets[str(i+1)]
                v[2].goal_position = targets[str(i+2)]
                i+=3

            print("avant  targets.items(): \n",  targets.items())
            robot.smooth_tick_read_and_write(3, verbose=False)
            print("Init position reached")
            time.sleep(2)
            print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
