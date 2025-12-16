# ROS 2 Workspace for TurtleBot3 Localization and Navigation

## Project Overview

This project is a ROS 2 workspace for developing and testing robot localization and navigation algorithms. It uses the TurtleBot3 robot platform in a Gazebo simulation environment. The workspace contains several custom ROS 2 packages for localization and motion planning, as well as standard TurtleBot3 simulation packages.

The main technologies used are:
- **ROS 2 Humble:** The core framework for robot software development.
- **Python:** The primary programming language for the custom ROS 2 packages.
- **Gazebo:** The simulation environment for the TurtleBot3 robot.
- **Nix:** For managing the development environment.

The key custom packages are:
- `lab04_pkg`: Implements an Extended Kalman Filter (EKF) for robot localization.
- `lab07_pkg`: Implements the Dynamic Window Approach (DWA) for local motion planning.

## Nix Environment

This project uses [Nix](https://nixos.org/) to manage the development environment. The `flake.nix` file at the root of the project defines all the necessary dependencies, including ROS 2 Humble, Gazebo, and other system packages.

To activate the development environment and prevent garbage collection of the environment, run the following command from the root of the project:

```bash
nix develop --profile ./nix-gc-root
```

This will drop you into a shell with all the required dependencies available.

The `flake.nix` also sets the `TURTLEBOT3_MODEL` environment variable to `burger`.

## Building and Running

### Building the Workspace

Once you are in the Nix development shell, you can build the ROS 2 workspace using the following command:

```bash
colcon build
```

### Sourcing the Workspace

The `flake.nix` file automatically sources the `install/local_setup.bash` file, so you don't need to source it manually.

### Running the Simulation

To run the TurtleBot3 simulation, you can use the launch files provided in the `turtlebot3_gazebo` package. For example, to launch the `turtlebot3_world`:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Running the Nodes

To run the custom nodes for localization and navigation, use the `ros2 run` command.

**EKF Localizer:**

```bash
ros2 run lab04_pkg localizer
```

**DWA Motion Planner:**

```bash
ros2 run lab07_pkg dwa_node
```

## Development and Testing

### Building, Testing, and Linting

- **Build all packages:** `colcon build`
- **Run all tests:** `colcon test`
- **Run tests for a specific package (e.g., `lab01_pkg`):** `colcon test --packages-select lab01_pkg`
- **Run a single Python test file (e.g., `test_flake8.py` in `lab01_pkg`):** `pytest src/lab01_pkg/test/test_flake8.py`
- **Lint Python code (Flake8):** `flake8 src/`
- **Lint Python docstrings (PEP 257):** `pydocstyle src/`

### Development Conventions

- The custom packages are Python-based ROS 2 packages using the `ament_python` build type.
- The code follows standard Python conventions (PEP 8 for formatting, PEP 257 for docstrings).
- Each package has a `package.xml` file that defines its dependencies and other metadata.
- Executable nodes are defined in the `entry_points` section of the `setup.py` file.
- Type hints should be used for function arguments and return values.