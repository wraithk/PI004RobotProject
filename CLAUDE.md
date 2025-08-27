# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MATLAB-based robotics project for the ENGN4627 course involving PenguinPi robots. The codebase supports both real hardware robots and simulation environments for developing autonomous robot behaviors including line following, landmark detection, and basic navigation.

## Architecture

### Core Components

- **PiBot.m**: Main robot interface class providing methods for motor control, sensor access, camera capture, and UI interaction
- **simulator/**: Contains `piBotSim.m` which simulates the PenguinPi robot for development and testing
- **arucoDetector/**: ArUco marker detection system for landmark recognition and pose estimation
- **CamCal/**: Camera calibration parameters and utilities

### Key Control Functions

- **inverse_kinematics.m**: Converts linear/angular velocities to individual wheel speeds
- **integrate_kinematics.m**: Forward kinematics for pose estimation
- **follow_line.m**: PD controller for line following with ArUco landmark detection
- **drive_square.m**: Geometric path planning for square trajectories
- **drive_circle.m**: Circular path execution

## Development Workflow

### Robot Setup

For simulation:
```matlab
addpath("simulator/");
pb = piBotSim("floor.jpg");  % or other floor images in simulator/
pb.place([x; y], theta);     % initial position/orientation
```

For real robot:
```matlab
pb = PiBot('192.168.50.1');  % replace with robot's IP address
```

### Camera Calibration

Camera parameters are stored in `CamCal/CamParam.mat`. The calibration images (1.png through 20.png) and calibration script (`calibrate_parameters.m`) are available for recalibration if needed.

### ArUco Detection

```matlab
addpath('arucoDetector/include');
addpath('arucoDetector');
addpath("arucoDetector/dictionary");
ArucoDict = load("arucoDict.mat");
CamParam = load("CamParam.mat");

[marker_nums, landmark_centres, marker_corners] = detectArucoPoses(
    img, marker_size, CamParam.cameraParams, ArucoDict.arucoDict);
```

### Control Architecture

The robot uses differential drive kinematics:
- Linear velocity `u` and angular velocity `q` are converted to wheel speeds via `inverse_kinematics(u, q)`
- Wheel speeds are commanded using `pb.setVelocity([left_wheel, right_wheel], duration)`
- Duration parameter is optional; without it, speeds persist until next command

## Common Patterns

### Real-time Control Loop

```matlab
while true
    img = pb.getImage();
    % Process image and compute control
    [u, q] = your_controller(img);
    [wl, wr] = inverse_kinematics(u, q);
    pb.setVelocity([wl, wr]);
    drawnow(); % Update visualizations
end
```

### PD Control Implementation

See `follow_line.m` for robust PD controller with:
- Exponential moving average for dt estimation
- Error derivative computation
- Speed scaling based on error magnitude
- Search behavior when line is lost

## File Conventions

- Main execution scripts: `main_*.m`
- Function files: lowercase with underscores
- Class files: CamelCase (e.g., `PiBot.m`)
- Calibration data: `.mat` files in appropriate subdirectories
- Floor images for simulation: `floor_*.jpg` in `simulator/`

## Important Notes

- Always use `addpath()` to include necessary directories
- Comment out `pb.place()` commands when transitioning from simulation to real robot
- Camera coordinate system: x forward, y left, z up
- Robot poses/velocities: [x; y] position vectors, scalar orientation theta
- Network timeouts limit command durations to <20 seconds
- Use `drawnow()` for real-time visualization updates