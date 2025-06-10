# Feedback-Linearization-of-a-Quadcopter-with-LQR
This project provides a Python simulation of a quadcopter controlled by a feedback linearization controller, demonstrating its trajectory tracking capabilities and control performance.

## Project Overview
This repository contains a high-fidelity Python simulation of a quadcopter, focusing on the implementation and analysis of a feedback linearization controller. The simulation models the quadcopter's nonlinear dynamics, including translational and rotational motion. The feedback linearization controller is designed to linearize the system, ensuring stable and accurate trajectory tracking. The project aims to provide an efficient simulation environment for understanding and validating nonlinear control techniques in UAV applications.

The simulation will generate the following plots and save them as PNG files in the project root directory:
-   `3DTrajectory.png`: 3D trajectory of the UAV’s flight path.
-   `PositionOverTime.png`: Position (x, y, z) over time.
-   `AttitudeOverTime.png`: Attitude angles (roll, pitch, yaw) over time.
-   `ControlInputs.png`: Control inputs (u1, u2, u3, u4) over time.
![3DTrajectory](https://github.com/user-attachments/assets/f859bdc6-ea7b-4c50-9c07-749148f42a1b)
![PositionOverTime](https://github.com/user-attachments/assets/7781571b-0dfd-41e8-8f48-0e0f356c7423)
![AttitudeOverTime](https://github.com/user-attachments/assets/ba0ceab4-fcbb-42e8-84d2-9fe90c2ef6b1)
![ControlInputs](https://github.com/user-attachments/assets/b2a72627-1b6c-4c55-9949-e84efe3a6180)
