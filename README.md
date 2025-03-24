# 🤖 Admittance Control for Reducing Human Physical Effort in Robot Programming through Motion Guidance

This repository contains Python scripts for experimental validation of **admittance control** using a **UR3 collaborative robot**. The main goal is to reduce the **physical effort** required by operators during linear trajectory programming, simulating dynamic behavior through force interaction.

It includes:

- **Control Robot**: Uses an admittance-based velocity controller (mass-spring-damper model) constrained to the **X-axis**, replacing freedrive with a more ergonomic and controllable method.
- **Disturbance Robot**: Applies **sinusoidal forces** to evaluate how the control robot responds under dynamic interaction.

---

## 📚 Table of Contents

1. [🧪 Research Context & Contributions](#-research-context--contributions)
2. [✨ Features](#-features)
3. [⚙️ Installation](#️-installation)
4. [🧪 Experimental Setup](#-experimental-setup)
5. [▶️ How to Run the Experiment](#-how-to-run-the-experiment)
6. [🎥 Demo Video & Images](#-demo-video--images)

---

## 🧪 Research Context & Contributions

This project supports a study on reducing operator effort in collaborative robot programming. Key contributions include:

- ✅ Demonstrated a **mass-based admittance control model** to reduce user-applied force.
- ✅ Showed **feasibility** of admittance control on UR3 (which lacks torque-level control).
- ✅ Provided a **low-complexity alternative** to adaptive and learning-based methods.
- ✅ Applied motion constraint to **X-axis** to isolate force interaction.
- ✅ Enabled comparison against UR’s native **freedrive mode**.

This framework was used in a peer-reviewed study focused on ergonomic, force-guided programming for collaborative robots.

---

## ✨ Features

### 🔧 Control Robot (UR3)
- Admittance-based velocity control
- Linear trajectory (X-axis only)
- Freedrive toggle
- Predefined motion poses
- Real-Time Control via RTDE
- Serial FSR sensor integration
- CSV data logging

### 🔎 Disturbance Robot (UR3)
- Sinusoidal force generation (Force Mode)
- Reactive pose control via FSR input
- Pose/state diagnostics
- Real-Time Control via RTDE
- Dashboard communication


---

## ⚙️ Installation

This project was developed and tested on **Ubuntu Linux 22.04 LTS**, where all dependencies were successfully installed and executed using Python 3.

Follow these steps to set up the environment and run the scripts on a machine connected to the UR3 robot.

### 1. Clone the Repository

```bash
git clone https://github.com/rosmerhyh/UR3_Admittance_Control.git
cd UR3_Admittance_Control
```

### 2. (Optional) Create a Virtual Environment
```bash
python3 -m venv ur3_env
source ur3_env/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```
The dependencies include:

- [`urx`](https://github.com/SintefManufacturing/python-urx): High-level UR robot control via socket interface
- [`ur-rtde`](https://pypi.org/project/ur-rtde/): Real-time control, data streaming, and force mode for UR robots
- [`pyserial`](https://pypi.org/project/pyserial/): Serial communication (used to read external sensor data)
- [`openpyxl`](https://pypi.org/project/openpyxl/): Export data to Excel format (optional)
- [`matplotlib`](https://matplotlib.org/stable/contents.html): Visualization of time-series data (force, velocity, acceleration)
- [`numpy`](https://numpy.org/doc/): Numerical computations

## 🔨 Experimental Setup

Two types of experiments were conducted:

### 🔹 Circuit to Read Force from FSR

A voltage divider with an Arduino measures force from an FSR sensor, sending values via serial to the control script.

<img src="graphic_resources/Circuit_Sensor.png" alt="Circuit sensor" width="100%"/>
*Figure 1: Circuit to read the force applied to the FSR sensor.*

---

### 🔹 Experiment 1: Without Disturbance

- One UR3 robot controlled by the user
- Tested control modes:
  - UR **Freedrive**
  - **Admittance control** with masses: `1 kg`, `5 kg`, `10 kg`
- Motion constrained to the X-axis (linear)
- Goal: evaluate force reduction vs. freedrive

<img src="graphic_resources/Experiment_Setup_1.png" alt="Experiment 1 setup" width="100%"/>
*Figure 2: Experimental setup for admittance control without disturbance.*

---

### 🔹 Experiment 2: With Disturbance (Sinusoidal Load)

- Two UR3 robots, mechanically connected
- Control Robot: same as in Experiment 1
- Disturbance Robot: applies sinusoidal load via Force Mode
- Goal: evaluate robustness of admittance control and freedrive under dynamic forces

<img src="graphic_resources/Experiment_Setup_2.png" alt="Experiment 2 setup" width="100%"/>
*Figure 3: Experimental setup for admittance control with disturbance.*

---

---

## ▶️ How to Run the Experiment

### 🧰 Equipment

- 2 Computers (one per UR3)
- 2 UR3 robots
- FSR sensor + Arduino
- 3D printed connection part
- Local network (manual IP config)

---

### 🧪 Step-by-Step

1. **Upload `force_sensor_FSR.ino`** to Arduino and connect it to **Computer 1**.
2. **Connect Computer 1** to the **Control UR3**, set IP, and test with option `25`.
3. **Run Freedrive** (option `2`) or **Admittance** (option `6`, adjust mass in script).
4. *(Optional)* Connect **Computer 2** to the **Disturbance UR3**, set IP.
5. Run `Disturbance_Robot.py`, select option `25` to go to start pose.
6. Mechanically connect both robots.
7. On Computer 2, start force injection with **option `9`**.
8. On Control Robot, run option `2` or `6` depending on test type.
9. Data is saved in `.csv` format for analysis.

---

## 🎥 Demo Video & Images

### 🔸 Video Demo

[![Watch the experiment 1](https://img.youtube.com/vi/90aXgfmtC50/hqdefault.jpg)](https://youtu.be/90aXgfmtC50)  
*Admittance control demo without external disturbance.*

---

### 🔸 Experiment Photos

<table>
  <tr>
    <td align="center">
      <img src="graphic_resources/volunteer_performing_experiment1_1.jpg" alt="Pic1" width="300"/><br>
      <sub><b>Photo 1:</b> Volunteer performing experiment 1</sub>
    </td>
    <td align="center">
      <img src="graphic_resources/volunteer_performing_experiment2_2.jpg" alt="Pic2" width="300"/><br>
      <sub><b>Photo 2:</b> Volunteer performing experiment 2</sub>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="graphic_resources/volunteer_performing_experiment2_3.jpeg" alt="Pic3" width="300"/><br>
      <sub><b>Photo 3:</b> Volunteer performing experiment 2</sub>
    </td>
    <td align="center">
      <img src="graphic_resources/volunteer_performing_experiment2_4.jpg" alt="Pic4" width="300"/><br>
      <sub><b>Photo 4:</b> Volunteer performing experiment 2</sub>
    </td>
  </tr>
</table>

---