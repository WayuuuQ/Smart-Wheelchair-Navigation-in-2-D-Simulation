# Smart Wheelchair Navigation Simulator

This repository contains a simple 2-D simulator for smart wheelchair
navigation. The simulator supports keyboard-based control, LiDAR
sensing, configurable environments using JSON scene files, and pluggable
controllers.

------------------------------------------------------------------------

## File Structure

    SMAR.../
    │
    ├── Simulator/
    │   ├── scenes/
    │   │   ├── s0.json
    │   │   ├── s1.json
    │   │   ├── s2.json
    │   │   ├── s3.json
    │   │   └── s4.json
    │   │
    │   ├── controller.py
    │   ├── environment.py
    │   ├── input_handler.py
    │   ├── main.py
    │   ├── renderer.py
    │   ├── sensor.py
    │   └── wheelchair.py
    │
    ├── .gitignore
    ├── environment.yml
    └── README.md

### File Description

-   **main.py** -- Entry point of the simulator that initializes
    components and runs the simulation loop.
-   **wheelchair.py** -- Implements the wheelchair kinematic model and
    updates the robot pose.
-   **input_handler.py** -- Converts keyboard inputs into user control
    commands.
-   **sensor.py** -- Simulates the LiDAR sensor and generates distance
    measurements to obstacles.
-   **controller.py** -- Contains the controller that computes the
    executed control command from user input and sensor data.
-   **environment.py** -- Loads scene configuration from JSON files and
    manages map boundaries, obstacles, and collision checks.
-   **renderer.py** -- Handles visualization of the environment,
    wheelchair, and optional LiDAR beams.
-   **scenes/** -- Contains JSON files defining different simulation
    environments.
-   **environment.yml** -- Defines the conda environment and project
    dependencies.
-   **.gitignore** -- Lists files that should not be tracked by Git.


## Running the Simulator

Run the simulator from the `Simulator` directory:

    python main.py --scene scenes/s1.json

Different scenes can be selected by changing the JSON file.


## Controls

  Key  | Action
  ----- |----------------------------
  ↑     |Move forward
  ↓     |Move backward
  ←     |Turn anticlockwise
  →     |Turn clockwise
  R     |Reset simulation
  L     |Toggle LiDAR visualization


## Setting Parameters

Runtime parameters can be configured through command-line arguments.

-   **--scene** : selects the environment JSON file to load.
-   **--ray_dens*** : sets the number of LiDAR rays used for
    sensing.
-   **--pedestrians*** : sets how many moving obstacles
    appear in the environment.
-   **--controller*** : selects the controller used to generate the
    control commands.

*To be implemented

These parameters allow different experimental setups without modifying
the simulator code.
