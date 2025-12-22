# 75551-Code

## About This Repository

Welcome to the code repository for **Team 75551**, featuring advanced robotics control systems for VEX Robotics Competition. This repository contains a comprehensive PROS-based control system implementing feedforward control, motion profiling, localization, and intelligent mechanism control.

This codebase represents a complete solution for competitive robotics, featuring:

- **Holonomic Drivetrain Control** with six-wheel kinematics
- **Feedforward Motor Control** with system identification
- **Motion Profiling** for smooth autonomous movements
- **Odometry-Based Localization** for precise position tracking
- **RAMSETE Path Following** for accurate trajectory tracking
- **Intelligent Intake System** with color-based sorting

---

## Features

### Current Implementations

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

- **Intake System with Color Sorting**  
  Two-roller intake system with automatic color detection and sorting based on alliance color. Supports multiple scoring heights with state-based control.

- **Motor Controller Abstraction**  
  High-level motor control with velocity bounds, deadband filtering, and acceleration limiting. Supports both voltage and acceleration-based control.

- **Simplex Linear Programming Solver**  
  Two-phase simplex method implementation for solving linear programming problems, useful for optimization tasks in robotics.

---

## Architecture

### Core Components

- **`drivetrain.hpp`** - Main drivetrain class managing six motors, kinematics, and control modes
- **`motor-controller.hpp`** - Individual motor control with feedforward and bounds checking
- **`ff-velocity-controller.hpp`** - Feedforward control law implementation
- **`motion-profiler.hpp`** - Velocity profile generation for smooth motion
- **`localization.hpp`** - Odometry-based pose estimation
- **`intake.hpp`** - Intake system with color sorting logic
- **`system-identification.hpp`** - Motor constant identification routines
- **`helper-functions.hpp`** - Mathematical utilities (sign, wrapToPi, sinc, etc.)
- **`structs.hpp`** - Data structures for wheels, rollers, poses, and motor states
- **`simplex.hpp`** - Linear programming solver

### Design Philosophy

This codebase emphasizes:

1. **Feedforward Control**: Using motor models to predict required voltages rather than relying solely on feedback
2. **Physical Constraints**: All control respects motor acceleration and velocity limits
3. **Modularity**: Each subsystem is self-contained with clear interfaces
4. **Documentation**: Comprehensive Doxygen comments for all public APIs

---

## Prerequisites

- **PROS Development Environment**  
  This project uses [PROS](https://pros.cs.purdue.edu/) for VEX V5 development. Ensure you have PROS installed and configured.

- **C++17 Compiler**  
  The code requires C++17 features (variants, constexpr, etc.). PROS includes a compatible toolchain.

- **GNU Make**  
  The project uses a Makefile for building. PROS provides this automatically.

- **(Optional) Nix**  
  The repository includes a `flake.nix` for reproducible development environments.

---

## Building and Deployment

### Standard PROS Workflow

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   cd 75551-Code
   ```

2. **Install PROS CLI** (if not already installed):
   ```bash
   # Follow instructions at https://pros.cs.purdue.edu/v5/getting-started/
   ```

3. **Build the Project**:
   ```bash
   make
   ```

4. **Upload to Robot**:
   ```bash
   pros upload
   ```

### Using Nix (Optional)

If you have Nix installed:

```bash
nix develop   # Enter development shell
make          # Build project
pros upload   # Upload to robot
```

---

## Configuration

### Motor Constants

Before using feedforward control, you must identify motor constants for each motor:

```cpp
// Run system identification
drivetrain.calculate_and_print_motor_constants();
```

This will output K_v, K_a, and K_s values for each motor. Update these in your initialization code.

### Physical Parameters

Configure these constants in `drivetrain.hpp` or your initialization:

- `wheelbase_length` - Distance between front and back wheels (meters)
- `trackwidth_length` - Distance between left and right wheels (meters)
- `wheel_radius` - Radius of wheels (meters)
- `gear_ratio` - Gear ratio of drivetrain
- `decimal_of_max_velocity` - Scaling factor for max velocity (safety)
- `decimal_of_max_acceleration` - Scaling factor for max acceleration (safety)

### Intake Configuration

In `intake.hpp`, configure color detection thresholds:

- `redHue` - Expected hue value for red blocks
- `blueHue` - Expected hue value for blue blocks
- `redHueUncertainty` - Tolerance for red detection
- `blueHueUncertainty` - Tolerance for blue detection

---

## Usage Examples

### Basic Drivetrain Control

```cpp
#include "drivetrain.hpp"

// Initialize drivetrain (example)
drivetrain dt(motors, wheelbase, trackwidth, motor_constants,
              linear_wheel, horizontal_wheel, imu, initial_pose);

// Tank drive control
void opcontrol() {
    while (true) {
        dt.tank_drive_control(0.01);  // 10ms control loop
        pros::delay(10);
    }
}
```

### Motion Profiling

```cpp
// Move forward 1 meter
dt.linear_mp(1.0);

// Rotate 90 degrees
dt.angular_mp(M_PI / 2.0);

// Move to a specific pose
pose target = {2.0, 1.0, M_PI / 4.0};
dt.mtp_mp(target);
```

### RAMSETE Path Following

```cpp
// Follow a path with RAMSETE correction
std::vector<differentialVels> velocities = {{0.5, 0.0}, {0.5, 0.1}, {0.3, 0.0}};
std::vector<pose> poses = {{1.0, 0.0, 0.0}, {2.0, 0.5, 0.2}, {2.5, 1.0, 0.3}};
dt.move_differential_robot_vels_ramsete(velocities, poses, 0.01);
```

### Intake Control

```cpp
#include "intake.hpp"

// Initialize intake
intake system(intake_motors, intake_constants);

// Enable color sorting
system.colorSorting = true;
system.allianceColor = red;

// Update intake (call in control loop)
void opcontrol() {
    while (true) {
        system.update_intake_state(0.01);
        pros::delay(10);
    }
}
```

### System Identification

```cpp
#include "system-identification.hpp"

// Identify constants for a set of motors
std::vector<MotorController> motors = {motor1, motor2, ...};
auto constants = SysIdent::calculate_Kv_Ka_and_Ks_s(motors);

// Or print directly
SysIdent::calculate_and_print_constants(motors);
```

---

## Documentation

All header files include comprehensive Doxygen documentation. To generate documentation:

```bash
doxygen Doxyfile  # If you have a Doxyfile configured
```

Or view the inline comments in the header files directly.

---

## Project Structure

```
75551-Code/
├── include/           # Header files
│   ├── drivetrain.hpp
│   ├── motor-controller.hpp
│   ├── ff-velocity-controller.hpp
│   ├── motion-profiler.hpp
│   ├── localization.hpp
│   ├── intake.hpp
│   ├── system-identification.hpp
│   ├── helper-functions.hpp
│   ├── structs.hpp
│   └── simplex.hpp
├── src/               # Source files
│   ├── main.cpp
│   ├── drivetrain.cpp
│   ├── motor-controller.cpp
│   └── ...
├── bin/               # Build output
├── firmware/          # PROS firmware files
├── Makefile           # Build configuration
├── project.pros       # PROS project file
└── README.md          # This file
```

---

## Future Plans

- **Pure Pursuit Path Following**: Alternative path following algorithm for comparison with RAMSETE
- **2D Motion Profiling**: Extended motion profiles for simultaneous translation and rotation
- **Path Generation Integration**: Support for importing paths from tools like Path Jerry
- **Enhanced Localization**: Integration with vision sensors for pose correction
- **Library Extraction**: Package reusable components as a standalone PROS library

---

## Contributing

Contributions, issues, and pull requests are welcome! When contributing:

1. Follow the existing code style and Doxygen comment format
2. Add documentation for new functions and classes
3. Test changes on hardware before submitting
4. Update this README if adding significant features

---

## License

This project uses PROS, which is licensed under the MPL 2.0. See the PROS documentation for details.

---

## Acknowledgments

- **PROS Development Team** for the excellent robotics framework
- **Purdue University ACM SIGBots** for PROS and educational resources
- The VEX Robotics community for inspiration and collaboration

---

## Contact

For questions, issues, or collaboration opportunities, please open an issue on the repository or contact the team directly.

---

**Note**: This code is designed for competitive robotics and should be thoroughly tested before use in competitions. Always verify motor constants, physical parameters, and control gains for your specific robot configuration.
