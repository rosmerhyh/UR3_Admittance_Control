
# ========================================================
#  DISTURBANCE ROBOT SCRIPT
#  PROJECT DEVELOPED BY: Rosmer Hasan Yepes Hoyos
#  INDUSTRIAL AUTOMATION ENGINEER | MASTER'S RESEARCH STUDENT
#  Universidad del Cauca
#  Email: rosmerh@unicauca.edu.co
# ========================================================

import urx  # Library for controlling Universal Robots (UR) via Python
import time  # Module for handling time-related functions, such as delays
import math  # Provides mathematical functions and constants
import rtde_control  # Module for real-time control of UR robots using RTDE protocol
import rtde_receive  # Module for receiving real-time data from UR robots via RTDE
import rtde_io  # Module for controlling input/output (IO) of UR robots via RTDE
import dashboard_client  # Provides an interface to the UR dashboard server for high-level commands
import csv  # Library for handling CSV file operations (reading/writing)
from openpyxl import Workbook  # Library for handling Excel files (XLSX format)
import matplotlib.pyplot as plt  # Library for plotting and visualizing data
import numpy as np  # Numerical computing library for handling arrays and mathematical operations
import warnings  # Module for managing warnings in Python
import threading  # Module for handling concurrent execution using threads
import sys  # System-specific functions and parameters
import serial  # Library for handling serial communication (e.g., with Arduino or sensors)


# ============================================
#  USING URX (NOT USED FOR EXPERIMENTS)
# ============================================

robot = urx.Robot("192.168.1.25",use_rt=True)
robot.set_tcp((0,0,0,0,0,0)) # Set TCP offset to zero
robot.set_payload(3,(0,0,0)) # Set payload to 3 kg

# Set motion parameters
a = 0.8 # Acceleration for robot movements
v = 0.1 # Velocity for robot movements

time.sleep(0.01) # Small delay to stabilize

# ========================================================
# USING UR_RTDE FOR ROBOT CONNECTION & INITIALIZATION
# ========================================================

# Initialize RTDE interface for receiving real-time data from the robot
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.25")

# Initialize RTDE interface for controlling the robot in real time
rtde_c = rtde_control.RTDEControlInterface("192.168.1.25")


# Initialize the Dashboard Client for high-level robot control (e.g., starting/stopping programs)
sc = dashboard_client.DashboardClient("192.168.1.25") #dashboard

def options(opt):
    """
    Function to execute different operations based on the selected option.
    :param opt: A string representing the selected option.
    """
    if opt == '1':
        # Move the robot to the basic initial position (Not the initial position for the experiments)
        pose = [-1.828118912373678, -1.598703523675436, -1.4817869663238525, -1.6207109890379847, 1.5374830961227417, 12.173362557088033]
        rtde_c.moveJ(pose,1,1,False) # Move in joint space with acceleration=1, velocity=1
        time.sleep(2) # Wait for the movement to complete
    elif opt == '3':
        # Enable freedrive mode, allowing the user to manually move the robot
        rtde_c.freedriveMode()
    elif opt == '4':
        # Disable freedrive mode, returning control to the robot's programmed movements
        rtde_c.endFreedriveMode()
    elif opt == '9':
        #Apply a sinusoidal disturbance force in the X direction using force mode

        import math # Import math module for sine and pi functions

        # Sinusoidal force parameters
        amp = 2        # Amplitude of the sinusoidal force [N]
        per = 5        # Period of the sine wave [s]
        tmin = 0       # Time when the force starts being applied
        tmax = 20      # Duration of the sinusoidal force application [s]

        def sin_force(t, amplitude, period, tmin, tmax):
            """
            Computes a sinusoidal force based on time, amplitude, and period.
            Returns zero if time is outside the [tmin, tmax] window.
            """
            if t <= tmin or t >= tmax:
                force = 0
            else:
                force = amplitude * math.sin(2*math.pi*t/period)
            return force
        
        # Force mode parameters
        limits = [0.1,0.1,0.5,0.5,0.5,0.5]   # Speed/force limits on each axis [x, y, z, Rx, Ry, Rz]
        task_frame = [0, 0, 0, 0, 0, 0]      # Task frame aligned with robot base
        selection_vector = [1, 0, 0, 0, 0, 0]    # Apply force in X direction only
        force_type = 2                           # Force frame is not transformed
        tn = 0                                   # Time variable for loop execution
        tactual = time.time()                    # Get current time for reference
        Ts = 1                                   # Sampling interval [s]

        while tn <= tmax:
            t_start = rtde_c.initPeriod()        # Start control period for RTDE loop
            tn = time.time() - tactual           # Elapsed time since start

            # Calculate the disturbance force to apply on X
            fx = -10 + sin_force(tn,amp,per,tmin,tmax)

            wrench = [fx, 0, 0, 0, 0, 0]  # Apply time-varying force in X axis
            rtde_c.forceMode(task_frame, selection_vector, wrench, force_type, limits)
            rtde_c.waitPeriod(t_start) # Wait until next period

        # Stop force mode and terminate the script
        rtde_c.forceModeStop()
        rtde_c.stopScript()


    elif opt == '12':
        #Read voltage from a sensor (e.g., FSR) and move the robot to different positions based on the pressure level detected
        
        # Open the serial port to read sensor data
        ser = serial.Serial('/dev/ttyACM0', 115200)
        def getvoltage():
            """
            Reads a line from the serial port, converts it to an integer,
            and then to volts by dividing by 1000.
            :return: voltage in volts
            """
            fsr_voltage = int(ser.readline().strip())  # Read and convert raw voltage
            voltage = fsr_voltage/1000
            return voltage

        while True:
            vol = getvoltage()  # Read current voltage from sensor

            if vol >= 4.8:
                # Low or no pressure detected
                print("Low or no pressure")
                pose = [-1.5321152845965784, -1.505298138861992, -1.5719056129455566, -1.660238882104391, 1.5195196866989136, 12.164671723042623]
                rtde_c.moveJ(pose,1,1,False)  # Move to pose corresponding to low pressure
                time.sleep(0.1)

            elif vol >= 3.6 and vol <4.8:
                # Medium pressure detected
                print("Medium pressure")
                pose = [-0.4751055876361292, -1.5053305190852662, -1.5716185569763184, -1.66021790126943, 1.5196194648742676, 12.16465646425356]
                rtde_c.moveJ(pose,1,1,False)  # Move to pose for medium pressure
                time.sleep(0.1)
            elif vol > 0 and vol < 3.6:
                # High pressure detected
                print("Presion alta")
                pose = [-2.5452776590930384, -1.5052815613201638, -1.5716547966003418, -1.6601869068541468, 1.5188360214233398, 12.164681736622946]
                rtde_c.moveJ(pose,1,1,False)  # Move to pose for high pressure
                time.sleep(0.1)

    elif opt == '25':
        #Moves the robot to a predefined initial pose for the experiments and retrieves various robot data

             
        time.sleep(1)

        # Define the initial joint position
        pose_ini = [-1.6039741675006312, -1.5993639431395472, -1.611436367034912, -1.5275772374919434, 1.563317894935608, 12.08038551012148]

        # Move in joint space with acceleration=1, velocity=1
        rtde_c.moveJ(pose_ini,1,1,False)

        time.sleep(1)

        # Retrieve the actual TCP pose (end-effector position and orientation)
        dat = rtde_r.getActualTCPPose()
        print(dat)
        print("-----------")

        # Retrieve the current joint positions
        dat1 = rtde_r.getActualQ()
        print(dat1)
        print("-----------")

        # Connect to the robot's dashboard client and retrieve the robot model
        sc.connect()
        dat2 = sc.getRobotModel()
        print(dat2)

    elif opt == '24':
        #Checks if the dashboard server connection to the robot is active. Prints the connection status as a boolean value.
        a = sc.isConnected() # Check if the dashboard client is connected to the robot
        print(a) # Print boolean result
        print(str(a)) # Print string representation of the result

while True:
    time.sleep(1)
    print("\nAvailable Options:")
    print("1. Move the robot to the initial (home) position.")
    print("3. Enable unlimited freedrive mode (manual guidance).")
    print("4. Disable freedrive mode.")
    print("9. Apply a sinusoidal disturbance force in the X direction (opposes main/control robot).")
    print("12. Read voltage from sensor and move to a predefined pose based on pressure level")
    print("25. Move to initial position for experiments and print robot status (pose, joints, model info).")
    print("24. Check if the robot dashboard server is connected.")
    print("q. Exit")

    chosen_option = input("Choose an option: ")

    if chosen_option == 'q':
        print("\nGoodbye! ('w')/")
        robot.close()
        rtde_c.stopScript()
        break

    options(chosen_option)