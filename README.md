# Constant Solver

## Purpose

This branch contains code and helpers for **solving controller constants**: it computes feedback gains (LQR-style) for a feedback controller **given** first-order feedforward constants for the plant model.

In short: you provide the feedforward model parameters (for example, the motor steady-state gain `K` and time constant `τ`) and the code solves for the optimal feedback gains using the LQR-style solvers used in *Controls Engineering in FRC*.

---

## Repository reference

This branch is based on the methods and solvers described in:

https://github.com/calcmogul/controls-engineering-in-frc/tree/main

---

## Key idea

1. Identify the feedforward (plant) constants for your system (e.g., `K` and `τ` for a first-order DC motor model).
2. Provide those feedforward constants to the solver.
3. The branch runs LQR-style numerical solvers (Riccati / state-feedback routines) consistent with the approaches in *Controls Engineering in FRC*.
4. The solver produces optimal feedback gains for a chosen quadratic cost (`Q`, `R`).
5. These feedback gains are intended to be used alongside the feedforward model in a combined FF + FB controller.

> The solver implementations and numerical approach used here are taken directly from the *Controls Engineering in FRC* material.

---

## Dependencies

### Eigen

This branch uses **Eigen**, a header-only C++ linear algebra library, for all matrix and vector operations.

### Installing Eigen

#### Ubuntu / Debian
```bash
sudo apt update
sudo apt install libeigen3-dev
```

#### macOS (Homebrew)
```bash
brew install eigen
```

#### Windows (vcpkg)
```bash
vcpkg install eigen3
```

#### Manual / Header-only install
You can also download Eigen directly and add the `Eigen/` directory to your compiler include path. Eigen does not require compilation.
