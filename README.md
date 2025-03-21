# WORK IN PROGRESS
----

# 🤖 Admittance Control for Reducing Human Physical Effort in Robot Programming through Motion Guidance

This repository contains Python scripts developed for experimental validation of **admittance control strategies** using a **UR3 collaborative robot**. The main goal is to reduce the **physical effort** required by human operators during the programming of linear trajectories, by simulating dynamic behaviors that guide robot motion through force interaction.

The system includes two main components:

- **Control Robot**: Implements an **admittance-based velocity controller** that simulates a **virtual mass-spring-damper system**. The controller restricts motion to a **single axis (X-axis)** and enables compliant, force-responsive movement during human-guided programming. This approach aims to replace the standard freedrive mode with a more controllable and ergonomic alternative, improving interaction quality and reducing user fatigue.

- **Disturbance Robot**: Applies controlled **sinusoidal forces** using the UR3's **force mode**, allowing the evaluation of the control robot’s responsiveness and robustness under external disturbances. It also includes functionality to react to sensor input (e.g., from an FSR) to simulate physical interaction or intentional interference.


---

## 🧪 Research Context & Contributions

This project supports a study on **minimizing human physical effort** in programming-by-demonstration for collaborative robots in industrial environments. The key contributions of this work include:

- ✅ Demonstrated the effectiveness of a **mass-based admittance control model** in reducing operator-applied force, particularly at **1 kg** and **5 kg** virtual mass settings.
- ✅ Validated the **feasibility of admittance control** in robots like the UR3, which do not support torque-level control required for classical impedance control.
- ✅ Compared the proposed method against adaptive and reinforcement-learning-based approaches, showing that a **fixed admittance model** can still achieve significant ergonomic improvements without added complexity.
- ✅ Replicated the concept of **constrained task spaces** by enforcing motion only along the X-axis, allowing for isolated and measurable force interaction.
- ✅ Provided a simpler, more accessible alternative to advanced adaptive methods, showing that **low-complexity controllers** can yield practical benefits in **stable, repetitive industrial tasks**.

This codebase has been used in a peer-reviewed research article focused on enhancing **human-robot interaction through force-guided motion**, contributing to the development of safer, more ergonomic collaborative robotics solutions.

---

## ✨ Features

This project provides a complete experimental framework for evaluating admittance control and external disturbance response in collaborative robots. The key features include:

### 🔧 Control Robot (Main UR3)
- **Admittance-Based Velocity Control**  
  Implements a velocity controller simulating a mass-spring-damper system to move the robot along the X-axis in response to user-applied force.

- **Linear Trajectory Execution**  
  Restricts movement to a single Cartesian axis to isolate interaction forces and facilitate analysis.

- **Predefined Pose Commands**  
  Includes options to move the robot to home or experimental starting positions for repeatable trials.

- **Freedrive Mode Toggle**  
  Allows switching between UR's built-in freedrive mode and the custom admittance control for comparison.

- **Force Sensor Integration**  
  Acquires external force data using a serial-connected piezoresistive sensor (e.g., FSR) to estimate interaction forces during motion.

- **Real-Time Data Logging**  
  Records time series data including target and actual velocity, applied force, estimated acceleration, and sensor voltage.

- **CSV Export for Analysis**  
  Automatically stores experimental data in CSV format for post-processing or visualization.

- **Visualization of Results**  
  Generates plots for:
  - TCP force over time
  - Desired vs. actual velocity
  - Desired vs. measured acceleration
  - Sensor voltages and estimated forces

---

### 🔎 Disturbance Robot (Secondary UR3)
- **Sinusoidal Force Injection**  
  Uses RTDE force mode to apply a sinusoidal disturbance force along the X-axis, simulating real-world interaction or interference.

- **Reactive Pose Control Based on Sensor Input**  
  Reads voltage values from a force sensor and adjusts the robot’s pose depending on detected pressure level (low, medium, high).

- **Real-Time Control via RTDE**  
  Executes force control in real time with synchronized sampling, allowing precise modulation of applied external forces.

- **Pose Initialization and State Monitoring**  
  Includes diagnostic options to print robot pose, joint angles, and robot model information from the UR dashboard.

- **Dashboard Connection Testing**  
  Verifies connection with the robot’s dashboard server for status and readiness checks.

---

## ⚙️ Installation

This project was developed and tested on **Ubuntu Linux 22.04 LTS**, where all dependencies were successfully installed and executed using Python 3.

Follow these steps to set up the environment and run the scripts on a machine connected to the UR3 robot.

### 1. Clone the Repository

```bash
git clone https://github.com/rosmerhyh/UR3_Admittance_Control.git
cd UR3_Admittance_Control