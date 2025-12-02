
# Lab 4: EKF Localization

This package contains the Python scripts for implementing an Extended Kalman Filter (EKF) for robot localization, as part of Lab 4.

## Objective

The main goal of this lab is to develop a ROS 2 package for localizing a TurtleBot3 robot using an EKF. The localization is based on landmark measurements and is improved by fusing data from odometry and an IMU.

## Files

This solution is divided into three main Python scripts:

- `task0.py`: A script to demonstrate and visualize the probabilistic motion and measurement models. It also shows how to compute the Jacobians symbolically.
- `localizer_node.py`: A ROS 2 node that implements the EKF for localization based on landmark measurements (Task 1).
- `extended_localizer_node.py`: A ROS 2 node that implements an extended EKF with a 5D state `[x, y, theta, v, w]` and fuses data from odometry and IMU (Task 2).

## Task 0: Probabilistic Models

The `task0.py` script can be run independently of ROS to understand the underlying models.

```bash
python3 src/lab04_code/task0.py
```

This script will:
1.  Generate and plot samples for the **velocity-based motion model** with different noise parameters. The plots are saved as `motion_model_samples.png`.
2.  Compute and print the symbolic Jacobians `G` (w.r.t. state) and `V` (w.r.t. control) for the motion model using `sympy`.
3.  Generate and plot samples for the **landmark measurement model**. The plot is saved as `measurement_model_samples.png`.
4.  Compute and print the symbolic Jacobian `H` (w.r.t. state) for the measurement model.

## Task 1: EKF Localizer Node

The `localizer_node.py` script contains the `LocalizerNode`, a ROS 2 node for EKF localization.

### State
The state is `mu = [x, y, theta]`.

### Subscriptions
- `/odom`: To get the current linear and angular velocities (`v`, `w`) for the prediction step.
- `/landmarks` (`landmark_msgs/msg/LandmarkArray`): To get the range and bearing of detected landmarks for the update step.

### Publications
- `/ekf` (`nav_msgs/msg/Odometry`): To publish the estimated pose and covariance of the robot.

### Logic
- **Prediction**: A timer runs at 20 Hz to trigger the EKF prediction step, using the velocity motion model.
- **Update**: The landmark callback triggers the EKF update step for each observed landmark. The landmark coordinates are hardcoded in the node.

## Task 2: Extended EKF Localizer Node

The `extended_localizer_node.py` script contains the `ExtendedLocalizerNode`, which is an enhanced version of the EKF.

### State
The state is extended to 5 dimensions: `mu = [x, y, theta, v, w]`.

### Subscriptions
- `/odom` (`nav_msgs/msg/Odometry`): Used as a measurement of `v` and `w` to correct the state.
- `/imu` (`sensor_msgs/msg/Imu`): Used as a measurement of `w` to correct the state.
- `/landmarks` (`landmark_msgs/msg/LandmarkArray`): Used for the landmark-based update, similar to Task 1, but with a 5D state.

### Publications
- `/ekf_extended` (`nav_msgs/msg/Odometry`): To publish the estimated pose, velocity, and their covariances.

### Logic
- **Prediction**: The prediction step uses a more complex motion model that predicts all 5 state variables.
- **Update**: The node performs an update step upon receiving messages from any of the three sensor sources (`/odom`, `/imu`, `/landmarks`), using the corresponding measurement model and Jacobian. This allows for sensor fusion to improve the state estimation.

## How to Run the Nodes

To run these nodes, you would typically build the ROS 2 workspace and then launch the nodes. As per the instructions, the package setup is not handled here. Assuming the package is correctly set up, you could run a node using:

```bash
# For Task 1
ros2 run <your_package_name> localizer_node

# For Task 2
ros2 run <your_package_name> extended_localizer_node
```
You would also need to have the simulation or the real robot running, publishing the `/odom`, `/imu`, and `/landmarks` topics.
