# 75551-Code

## About This Repository

Welcome to the code repository for **Team 75551A**, featuring advanced robotics control systems for VEX Robotics Competition. This repository contains a comprehensive PROS-based control system implementing feedforward control, motion profiling, localization, and intelligent mechanism control.

---

## Current Implementations

- **Feedforward Velocity Control**  
  DC motor control using feedforward constants (K_v, K_a, K_s) for precise velocity and acceleration control without requiring feedback loops.

- **System Identification**  
  Automated motor characterization that determines feedforward constants through steady-state and acceleration testing. Supports multiple motors simultaneously.

- **Holonomic Drivetrain**  
  Six-wheel holonomic drivetrain (4 mecanum + 2 omni wheels) with differential drive kinematics, velocity bounds calculation, and acceleration-limited control.

- **Motion Profiling**  
  Trapezoidal and triangular motion profiles for linear and angular movement with configurable acceleration and velocity limits.

- **RAMSETE Controller**  
  Nonlinear feedback controller for path following that provides stable convergence to desired poses and velocities, even with modeling errors.

- **Odometry-Based Localization**  
  Position tracking using rotation sensors and IMU, with support for pose reset and continuous pose updates.

- **Motor Controller Abstraction**  
  High-level motor control with velocity bounds, deadband filtering, and acceleration limiting. Supports both voltage and acceleration-based control.

---

## Future Plans

- **2D Motion Profiling**: Extended motion profiles for simultaneous translation and rotation
- **Path Generation Integration**: Support for importing paths from tools like Path Jerry
- **Enhanced Localization**: Integration with vision sensors for pose correction
- **Library Extraction**: Package reusable components as a standalone PROS library
