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