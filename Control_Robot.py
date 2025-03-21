
# ========================================================
#  CONTROL ROBOT SCRIPT
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
#  USING URX (NOT NECESSARY FOR EXPERIMENTS)
# ============================================

# Connect to the UR robot at the specified IP address using the URX library with RTDE enabled
robot = urx.Robot("192.168.1.29", use_rt=True)

# Set the Tool Center Point (TCP) offset to zero (default position)
robot.set_tcp((0, 0, 0, 0, 0, 0))

# Set the payload of the robot to 1 kg, with the center of mass at (0, 0, 0)
robot.set_payload(1, (0, 0, 0))

# Define motion parameters
a = 0.8  # Acceleration for robot movements
v = 0.1  # Velocity for robot movements

# Short delay to allow initialization
time.sleep(0.01)

# ========================================================
# USING UR_RTDE FOR ROBOT CONNECTION & INITIALIZATION
# ========================================================

# Initialize RTDE interface for receiving real-time data from the robot
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.29")

# Initialize RTDE interface for controlling the robot in real time
rtde_c = rtde_control.RTDEControlInterface("192.168.1.29")

# Initialize the Dashboard Client for high-level robot control (e.g., starting/stopping programs)
sc = dashboard_client.DashboardClient("192.168.1.29")

def options(opt):
    """
    Function to execute different operations based on the selected option.
    :param opt: A string representing the selected option.
    """
    if opt == '1': 
        # Move the robot to the basic initial position (Not the initial position for the experiments)
        pose = [-1.828118912373678, -1.598703523675436, -1.4817869663238525, -1.6207109890379847, 1.5374830961227417, 12.173362557088033]
        rtde_c.moveJ(pose, 1, 1, False)  # Move in joint space with acceleration=1, velocity=1
        time.sleep(2)  # Wait for the movement to complete
    elif opt == '2':

        import time
        time.sleep(3) # Short delay before starting the process

        def getforces(rtde_r):
            """
            Retrieves the actual TCP force from the robot.
            :param rtde_r: RTDEReceiveInterface object to get sensor data.
            :return: List containing forces (X, Y, Z) and torques (X, Y, Z).
            """
            fdat = rtde_r.getActualTCPForce()
            return fdat

        def freedrive_mode_test(rtde_r,rtde_c,ser):
            """
            Enables freedrive mode on the robot, allowing manual guidance while collecting force sensor data.
            :param rtde_r: RTDEReceiveInterface object for reading forces.
            :param rtde_c: RTDEControlInterface object to control the robot.
            :param ser: Serial object for reading sensor data.
            :return: Lists of forces, torques, sensor voltages, and timestamps.
            """
            print("Entering freedrive mode...")
            free_axes = [1, 0, 0, 0, 0, 0] # Allow movement only in the X-axis
            feature = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # No special features applied
            rtde_c.freedriveMode(free_axes, feature)
            start_time = time.time() # Start time tracking

            # Initialize lists to store data
            tcp_forces_gen = []
            fx, fy, fz = [], [], []
            tx, ty, tz = [], [], []
            voltages = []
            fsensor = []
            time_list = []

            # Define experiment duration
            T = 10  # Total duration in seconds
            Ts = 0.5  # Sampling interval in seconds
            t = 0   # Initial time


            while t <= T:
                t_start = time.time()
                while time.time() - t_start <= Ts:
                    pass # Maintain sampling period

                print(f"Second {t}: ")

                # Retrieve force values from robot
                f = getforces(rtde_r)

                # Read sensor voltage from serial port
                fsr_voltage = ser.readline().strip()
                try:
                    fsr_voltage = int(fsr_voltage)
                except:
                    fsr_voltage = 5000 # Default fallback value if reading fails

                # Convert voltage to force using a polynomial model
                voltage = fsr_voltage/1000
                force = -2.0837*(voltage**3) + 25.442*(voltage**2) -104.76*voltage + 147.59

                # Store force data
                tcp_forces_gen.append(f)
                fx.append(f[0])
                fy.append(f[1])
                fz.append(f[2])
                tx.append(f[3])
                ty.append(f[4])
                tz.append(f[5])
                voltages.append(voltage)
                fsensor.append(force)

                # Print force values
                print(f"Effector forces: {f}")

                time_list.append(t)
                t += Ts

            total_time = time.time() - start_time
            print(f"Exiting freedrive mode after {total_time} seconds.")
            rtde_c.endFreedriveMode() # Disable freedrive mode

            print("\n")
            print(tcp_forces_gen)
            print("\n")

            return tcp_forces_gen,fx,fy,fz,tx,ty,tz,time_list,voltages,fsensor
        
        def create_table_csv(data, name):
            """
            Saves collected data into a CSV file.
            :param data: List of tuples containing time, force_x, and force_sensor values.
            :param name: Name of the CSV file.
            """
            with open(name, 'w', newline='') as file_csv:
                writer_csv = csv.writer(file_csv)
                # Write headers
                writer_csv.writerow(['Time','Force_x','Force_Sensor'])
                # Write data
                for fila in data:
                    writer_csv.writerow(fila)
        
        # Initialize serial connection to the sensor
        ser = serial.Serial('/dev/ttyACM0', 115200) 

        # Run freedrive mode and collect data
        full,f1,f2,f3,t1,t2,t3,all_time,volt,sensor_forc = freedrive_mode_test(rtde_r,rtde_c,ser)
        
        # Prepare data and save to CSV
        data = list(zip(all_time,f1,sensor_forc))
        name_file_csv = "Test_Option_2.csv"
        create_table_csv(data, name_file_csv)
        print("CSV file successfully created:", name_file_csv)

        # ============================================
        #           PLOTTING RESULTS
        # ============================================

        # Plot TCP force in X direction
        plt.plot(all_time,f1, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Force TCP in X')
        plt.grid(True)
        plt.show()

        # Plot TCP force in Y direction
        plt.plot(all_time,f2, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Force TCP in Y')
        plt.grid(True)
        plt.show()

        # Plot TCP force in Z direction
        plt.plot(all_time,f3, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Force TCP in Z')
        plt.grid(True)
        plt.show()

        # Plot TCP torque in X direction
        plt.plot(all_time,t1, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Torque TCP in X')
        plt.grid(True)
        plt.show()

        # Plot TCP torque in Y direction
        plt.plot(all_time,t2, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Torque TCP in Y')
        plt.grid(True)
        plt.show()

        # Plot TCP torque in Z direction
        plt.plot(all_time,t3, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Torque TCP in Z')
        plt.grid(True)
        plt.show()

        # Plot sensor voltage readings
        plt.plot(all_time,volt, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Sensor voltages')
        plt.grid(True)
        plt.show()   

        # Plot force measured by the sensor
        plt.plot(all_time,sensor_forc, marker='o')
        plt.xlabel('Time (s)')
        plt.ylabel('Sensor forces')
        plt.grid(True)
        plt.show() 

    elif opt == '3':
        # Enable freedrive mode, allowing the user to manually move the robot
        rtde_c.freedriveMode()
    elif opt == '4':
        # Disable freedrive mode, returning control to the robot's programmed movements
        rtde_c.endFreedriveMode()
    elif opt == '6':
        #This option implements an admittance control model using a virtual mass-spring-damper system. The system is discretized and executed in real-time, reading forces from the sensor FSR to adjust the robot’s motion dynamically.


        # ============================================
        #       SYSTEM PARAMETERS INITIALIZATION
        # ============================================

        import time # Import time module for delays and real-time execution
        
        # Experiment settings
        mass =10 # Virtual mass for the admittance control model
        m =10 # Duplicated mass variable for plots
        T = 10 # Total duration of the movement in seconds
        Vmax = 0.1 # Maximum velocity in X direction
        settling_time = 2 # Settling time for system response

        # Sampling time settings
        Ts = 0.5 # Discrete-time step in seconds
        t = 0  # Initialize time counter

        '''
        ABCD
        A: 
            1 -> With load
            2 -> Without load
        B: 
            1 -> Free Drive
            2 -> m = 1
            3 -> m = 5
            4 -> m = 10
        C:
            1 -> Volunteer 1
            2 -> Volunteer 2
            3 -> Volunteer 3
            4 -> Volunteer 4
        D:
            1 -> Attempt 1
            2 -> Attempt 2
            3 -> Attempt 3
            4 -> Attempt 4
        '''

        text = "2211"
        font = 12
        force_opo = 'sin -8 AND -12 '

        # Storage lists for collected data
        forces_fsr_before = [] # Force values from the sensor
        forces_fsr_after = [] # Force values from the sensor (Not used)
        volt_time = [] # Voltage values from the sensor
        wrong_value = ''
        trajectory_forces = [] # Forces applied along the trajectory
        trajectory_speed = [] # Measured velocity of the robot
        time_t = [] # Time samples
        targ_velocity = [] # Target velocity commanded to the robot
        desired_acceleration = [] # Desired acceleration
        measured_acceleration = [] # Measured acceleration from the robot

        # Short delay before starting
        time.sleep(0.1)

        # Initialize serial communication with the sensor (FSR sensor)
        ser = serial.Serial('/dev/ttyACM0', 115200)
        ser.flush()


        def exp(x):
            """
            Computes the exponential function e^x.

            :param x: The exponent value.
            :return: The result of e raised to the power of x.
            """
            return math.exp(x)
        
        # ============================================
        #       ADMITTANCE CONTROL FUNCTION
        # ============================================

        def admittance(force, sampling_time,mass):
            """
            Implements the discrete-time admittance control model.

            :param force: External force applied to the system (FSR Sensor).
            :param sampling_time: The discrete-time step for numerical calculations.
            :param mass: Virtual mass used in the model.
            :return: Computed velocity and acceleration.
            """
            m = mass
            damping = 1.05 # Ensures stability (must be > 1 to avoid imaginary values)
            settling_time = 2 # Settling time
            beta = 8*m / settling_time # Viscous friction coefficient
            k = 16*m / (damping * settling_time)**2 # Stiffness
            h = sampling_time # Sampling time (discretization step)

            # Initialize state-space coefficients only once
            if not hasattr(admittance, "x1"):
                admittance.a11 = (beta*exp(-(beta*h - h*(beta**2 - 4*k*m)**(1/2))/(2*m)) - beta*exp(-(beta*h + h*(beta**2 - 4*k*m)**(1/2))/(2*m)) + exp(-(beta*h + h*(beta**2 - 4*k*m)**(1/2))/(2*m))*(beta**2 - 4*k*m)**(1/2) + exp(-(beta*h - h*(beta**2 - 4*k*m)**(1/2))/(2*m))*(beta**2 - 4*k*m)**(1/2))/(2*(beta**2 - 4*k*m)**(1/2))
                admittance.a12 = -(m*exp(-(h*(beta + (beta**2 - 4*k*m)**(1/2)))/(2*m)) - m*exp(-(h*(beta - (beta**2 - 4*k*m)**(1/2)))/(2*m)))/(beta**2 - 4*k*m)**(1/2)
                admittance.a21 = (k*exp(-(h*(beta + (beta**2 - 4*k*m)**(1/2)))/(2*m)) - k*exp(-(h*(beta - (beta**2 - 4*k*m)**(1/2)))/(2*m)))/(beta**2 - 4*k*m)**(1/2)
                admittance.a22 =(beta*exp(-(h*(beta + (beta**2 - 4*k*m)**(1/2)))/(2*m)) + exp(-(h*(beta + (beta**2 - 4*k*m)**(1/2)))/(2*m))*(beta**2 - 4*k*m)**(1/2) - beta*exp(-(h*(beta - (beta**2 - 4*k*m)**(1/2)))/(2*m)) + exp(-(h*(beta - (beta**2 - 4*k*m)**(1/2)))/(2*m))*(beta**2 - 4*k*m)**(1/2))/(2*(beta**2 - 4*k*m)**(1/2))
                admittance.b1 = -(beta*exp(-(beta*h - h*(beta**2 - 4*k*m)**(1/2))/(2*m)) - beta*exp(-(beta*h + h*(beta**2 - 4*k*m)**(1/2))/(2*m)) + exp(-(beta*h + h*(beta**2 - 4*k*m)**(1/2))/(2*m))*(beta**2 - 4*k*m)**(1/2) + exp(-(beta*h - h*(beta**2 - 4*k*m)**(1/2))/(2*m))*(beta**2 - 4*k*m)**(1/2) - 2*(beta**2 - 4*k*m)**(1/2))/(2*k*(beta**2 - 4*k*m)**(1/2))
                admittance.b2 = -(exp(-(beta*h + h*(beta**2 - 4*k*m)**(1/2))/(2*m)) - exp(-(beta*h - h*(beta**2 - 4*k*m)**(1/2))/(2*m)))/(beta**2 - 4*k*m)**(1/2)
                admittance.x1 = 0 # Initial position state
                admittance.x2 = 0 # Initial velocity state

            # State space model of the admittance
            # Compute the next states
            x1_next = admittance.a11 * admittance.x1 + admittance.a12 * admittance.x2 + admittance.b1 * force
            x2_next = admittance.a21 * admittance.x1 + admittance.a22 * admittance.x2 + admittance.b2 * force

            # Output is velocity
            # Compute velocity and acceleration
            velocity = admittance.x2
            acceleration = (x2_next - velocity) / h


            # Update states
            admittance.x1 = x1_next
            admittance.x2 = x2_next

            # Return velocity
            return velocity, acceleration

        def get_terminal_forces(rtde_r):
            """
            Retrieves the force applied at the end-effector (TCP) in the X direction.

            :param rtde_r: RTDEReceiveInterface object to get real-time data from the robot.
            :return: The force in the X-axis at the TCP.
            """
            data = rtde_r.getActualTCPForce()
            terminal_forces = data[0] # Extract force in X-axis
            return terminal_forces
        
        def get_terminal_speed_x(rtde_r):
            """
            Retrieves the current Cartesian velocity in the X direction.

            :param rtde_r: RTDEReceiveInterface object to get real-time data from the robot.
            :return: The current velocity in the X direction [m/s].
            """
            data = rtde_r.getActualTCPSpeed()
            cartesian_speed = data[0] # Extract velocity in X-axis
            return cartesian_speed
        
        def get_target_speed_x(rtde_r):
            """
            Retrieves the target velocity in the X direction (the velocity command sent to the robot).

            :param rtde_r: RTDEReceiveInterface object to get target velocity.
            :return: The target velocity in the X direction [m/s].
            """
            data = rtde_r.getTargetTCPSpeed()
            target_velocity = data[0]
            return target_velocity
        
        def get_accelerations(rtde_r):
            """
            Retrieves the current acceleration of the end-effector in the X direction.

            :param rtde_r: RTDEReceiveInterface object to get real-time acceleration data.
            :return: The acceleration in the X-axis [m/s²].
            """
            data = rtde_r.getActualToolAccelerometer() 
            acceleration_x1 = data[0] # Extract acceleration in X-axis
            return acceleration_x1
        
        def create_table_csv(data, name):
            """
            Saves collected experimental data into a CSV file.

            :param data: List of tuples containing recorded data points.
            :param name: Name of the CSV file to save.
            """
            with open(name, 'w', newline='') as file_csv:
                writer_csv = csv.writer(file_csv)

                # Write column headers
                writer_csv.writerow(['Time','Desired_Speed', 'Measured_Speed', 'Desired_Acceleration','Measured_Acceleration','Terminal_Force','Serial_Voltage','Forces_Before','Forces_After'])

                # Write data rows
                for fila in data:
                    writer_csv.writerow(fila)

        
        def calculate_velocity(t):
            """
            Computes the desired velocity profile based on time.

            :param t: Current time step [s].
            :return: Desired velocity in the X direction [m/s].
            """
            return (4 * Vmax * t / T) * (1 - (t / T))
        
        
        def calculate_acceleratio(t):
            """
            Computes the desired acceleration profile based on time.

            :param t: Current time step [s].
            :return: Desired acceleration in the X direction [m/s²].
            """
            return (4 * Vmax / T) - ((8 * Vmax * t) / T**2)
        
        
        def calculate_force():
            """
            Reads and converts the force sensor (FSR) voltage into force using a polynomial approximation.

            :return: Estimated force [N].
            """
            fsr_voltage = ser.readline().strip() # Read raw voltage from serial port
            print(fsr_voltage)

            try:
                fsr_voltage = int(fsr_voltage)
            except:
                fsr_voltage = 5000 # Default value if conversion fails
                force = 0
                forces_fsr_before.append(force)
                return force

            # Convert voltage to force using a polynomial function
            #To generate the polynomial function, a characterization of the sensor was made using forces applied to the sensor by means of the ur3 robot.
            volt_time.append(fsr_voltage)
            voltage = fsr_voltage/1000 # Convert to volts
            force = -2.0837*(voltage**3) + 25.442*(voltage**2) -104.76*voltage + 147.59

            # Store force value
            forces_fsr_before.append(force)
            return force
        

        # ============================================
        #       EXPERIMENT LOOP (REAL-TIME SIMULATION)
        # ============================================

        while t <= T:
            t_start = time.time()
            if t <= T:
                # Read force from sensor
                force = calculate_force()
                # Compute velocity using admittance model
                velocity_x,accel= admittance(force,Ts,mass)
            else:   
                velocity_x = 0
                accel = 0
            # Ensure velocity remains within allowed limits
            velocity_x = max(0,velocity_x)
            velocity_x = min(Vmax,velocity_x)
            
            # Send velocity command to the robot
            velocity_cartesian = [velocity_x, 0.0, 0.0, 0.0, 0.0, 0.0]
            rtde_c.speedL(velocity_cartesian, max(0,accel), Ts)
            while time.time() - t_start <= Ts:
                pass

            # Store measured forces and velocities
            terminal_forces = get_terminal_forces(rtde_r)
            trajectory_forces.append(terminal_forces)

            terminal_speed = get_terminal_speed_x(rtde_r)
            trajectory_speed.append(terminal_speed)

            tar_velocity = get_target_speed_x(rtde_r)
            targ_velocity.append(tar_velocity)

            accel_meas = get_accelerations(rtde_r)
            measured_acceleration.append(accel_meas)
            
            desired_acceleration.append(accel)

            time_t.append(t)
            t = t + Ts 

        # Stop robot movement
        time.sleep(0.5)
        rtde_c.speedStop()

        # ============================================
        #       SAVE DATA TO CSV
        # ============================================

        data = list(zip(time_t,targ_velocity, trajectory_speed,desired_acceleration,measured_acceleration,trajectory_forces,volt_time,forces_fsr_before))
        name_file_csv = "Data_{}.csv".format(text)
        create_table_csv(data, name_file_csv)
        print("CSV file created successfully:", name_file_csv)
        
   
        print(volt_time)
        print("---------")
        print(forces_fsr_before)
        print("---------")
        print(forces_fsr_after)

        # ============================================
        #       PLOTTING RESULTS
        # ============================================

        plt.plot(time_t, trajectory_forces, 'bo-', label='Force in x axis')
        plt.xlabel('Time [s]',fontsize=font)
        plt.ylabel('Force [N]',fontsize=font)
        plt.title('Force on the x axis of the terminal in the robot with speed control',fontsize=font)
        plt.legend()
        plt.grid(True)
        plt.show()
        plt.figure()

        plt.plot(time_t, targ_velocity, label='Desired velocity', color='blue', linestyle='-')
        plt.plot(time_t, trajectory_speed, label='Measured velocity', color='red', linestyle='-')
        plt.xlabel('Time [s]', fontsize=font)
        plt.ylabel('Velocity [m/s]', fontsize=font)
        plt.title("Velocity when force in admittance funtion has a m = {} and settling_time = {}".format(m,settling_time), fontsize=font)
        plt.grid(True)
        plt.legend()
        plt.show()

        plt.figure()
        plt.plot(time_t, desired_acceleration, label='Desired acceleration', color='blue', linestyle='-')
        plt.plot(time_t, measured_acceleration, label='Measured acceleration', color='red', linestyle='-')
        plt.xlabel('Time [s]', fontsize=font)
        plt.ylabel('Acceleration [m/s^2]', fontsize=font)
        plt.title("Acceleration in the x axis of the terminal when force in admittance funtion has a m = {} and settling_time = {}".format(m,settling_time), fontsize=font)
        plt.grid(True)
        plt.legend()
        plt.show()
        
        plt.figure()
        plt.plot(time_t, volt_time, 'bo-', label='Voltage in serial')
        plt.xlabel('Time [s]',fontsize=font)
        plt.ylabel('Millivolts [mV]',fontsize=font)
        plt.title('Voltage in mV in the serial port',fontsize=font)
        plt.legend()
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(time_t, forces_fsr_before, 'bo-', label='FSR Before')
        plt.xlabel('Time [s]',fontsize=font)
        plt.ylabel('Force [N]',fontsize=font)
        plt.title('Forces using the polyfit',fontsize=font)
        plt.legend()
        plt.grid(True)
        plt.show()

    elif opt == '11':
        #Reads the voltage value from a serially connected sensor (FSR). The voltage is converted from millivolts to volts and printed.

        # Open the serial port connection
        ser = serial.Serial('/dev/ttyACM0', 9600)
        def call():
            """
            Reads a voltage value from the serial port.

            :return: Voltage in volts.
            """
            fsr_voltage = int(ser.readline().strip()) # Read raw voltage from sensor
            voltage = fsr_voltage/1000 # Convert from millivolts to volts
            return voltage
        
        # Call function to read and print sensor data
        call()
        dat = call()
        print(dat)

    elif opt == '25':
        #Moves the robot to a predefined initial pose for the experiments and retrieves various robot data

        # Define the initial joint position
        pose_ini = [-1.1733711401568812, -1.4588204112700005, -1.7440412044525146, -1.5385695782354851, 1.5759679079055786, 12.510759655629293]

        # Move in joint space with acceleration=1, velocity=1
        rtde_c.moveJ(pose_ini,1,1,False)

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
    time.sleep(0.5)
    print("\nAvailable Options:")
    print("1. Move the robot to the initial (home) position.")
    print("2. Free movement (Freedrive) along the X-axis for 10 seconds with force and torque sensing at the TCP (Results plotted).")
    print("3. Enable unlimited freedrive mode (manual movement mode).")
    print("4. Disable freedrive mode.")
    print("6. Speed control using the admittance control function with force on the X-axis, velocity, and acceleration graphs over time.")
    print("11. Read and display the voltage of a sensor connected via the serial port.")
    print("24. Check the robot server connection (command test).")
    print("25. Move the robot to the initial position to execute option 2 or 6 with or without an opposing force.")
    print("q. Exit")

    chosen_option = input("Choose an option: ")

    if chosen_option == 'q':
        print("\nGoodbye! ('w')/")
        robot.close()
        rtde_c.stopScript()
        break

    options(chosen_option)