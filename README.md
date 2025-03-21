# 🤖 Admittance-Based Control and Disturbance Robot (UR3)

This project contains Python scripts developed for experimental validation of admittance-based control strategies using a Universal Robots UR3 collaborative robotic arm. The main objective is to reduce the physical effort required by a human operator during the programming of linear robot trajectories through physical guidance.

It includes two main components:

- **Control Robot**: Implements an **admittance control strategy** to guide the UR3 robot along a linear trajectory in the X-axis. The controller simulates a virtual **mass-spring-damper system** that governs the robot’s velocity in response to external forces applied by the user. This approach allows the robot to move compliantly and naturally, reducing the physical effort required for demonstration-based programming. The admittance control replaces the conventional freedrive mode with a dynamic behavior that can be tuned by adjusting the virtual inertia parameters. The user pushes the robot to teach a motion path, and the controller ensures that the resulting motion is smooth, responsive, and physically efficient.

- **Disturbance Robot**: Simulates dynamic external disturbances that interact with the control robot’s motion. It applies controlled sinusoidal forces along the same axis to evaluate how different admittance configurations respond to unexpected or interfering forces during programming tasks.

The entire interaction takes place along a **linear trajectory**, which allows for precise analysis of velocity and force responses under different virtual mass values. The system integrates force sensors (e.g., FSR) and provides real-time data logging and plotting of key metrics such as applied force, measured velocity, and acceleration.

This setup supports direct comparison between admittance control and the standard freedrive mode in collaborative robotics. The scripts were used as part of a research study focused on improving human-robot interaction in industrial settings through intuitive and low-effort programming techniques.
