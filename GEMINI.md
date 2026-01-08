# Gemini Project Overview: C++ Robotics Simulation Library

## Project Overview

This project is a C++ library designed for the simulation and analysis of robotic arms. The name "lib_mophongrobot" (from the README and Vietnamese for "robot simulation library") suggests its primary purpose.

The core of the library revolves around the concepts of robotic kinematics, using Denavit-Hartenberg (DH) parameters to define robot links and transformation matrices to calculate the robot's position and orientation.

The main components are:
*   A `CRobot` class representing a 4-link robotic arm.
*   A `Clink` class for individual robot links defined by DH parameters.
*   A comprehensive `Matrix` library for linear algebra operations, which is fundamental for the kinematics calculations.
*   A `Transformations` module to handle 3D geometric transformations (rotation, translation).

The library provides functionality for:
*   **Forward Kinematics**: Calculating the end-effector's position from given joint angles.
*   **Inverse Kinematics**: Calculating the required joint angles to reach a desired end-effector position.
*   Trajectory planning and generation.

## Building and Running

**TODO:** No build scripts (e.g., Makefile, CMakeLists.txt) were found in the project. To compile this project, you will likely need to set up a C++ compiler and manually compile the `.cpp` source files, linking them together to create a library or an executable.

Example (conceptual):
```bash
g++ -c *.cpp
g++ -o robot_simulation *.o
```

## Development Conventions

*   **Language**: The project is written in C++.
*   **Coding Style**: The code seems to follow a class-based, object-oriented structure. Header files (`.h`) are used to declare classes and functions, while implementation is in the corresponding `.cpp` files.
*   **Comments**: The source code contains comments in both English and Vietnamese, which should be noted when making modifications.

## Key Files

*   `CRobot.h` / `CRobot.cpp`: Defines the main `CRobot` class. This class aggregates the robot links and orchestrates the kinematics calculations. It represents a complete robotic arm.
*   `Clink.h` / `Clink.cpp`: Defines the `Clink` class, which represents a single link of a robot using the Denavit-Hartenberg (DH) convention. It is the building block for the `CRobot`.
*   `Matrix.h` / `Matrix.cpp`: A custom linear algebra library. It provides `matrix`, `smatrix` (square matrix), and `vectorm` classes with overloaded operators for common matrix and vector operations. This is a critical dependency for all transformation and kinematics math.
*   `Transformations.h` / `Transformations.cpp`: A utility module that provides functions to create 4x4 homogeneous transformation matrices for 3D scaling, rotation, and translation.
*   `NumMethod.h` / `NumMethod.cpp`: Likely contains numerical methods used in the kinematics solvers, although it was not analyzed in detail.
