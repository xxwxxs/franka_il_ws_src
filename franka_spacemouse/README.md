# Franka Spacemouse

This repository contains ROS 2 packages for controlling a Franka FR3 arm with the Franka Hand using a 3Dconnexion SpaceMouse. 

## Packages

### 1. `franka_arm_controllers`
This package provides a Joint Impedance controller for Franka arms. It subscribes to a target cartesian velocity input and sends torque commands to the robot.

#### Key Features:
- Implements a `JointImpedanceController` for controlling the robot's torques.
- Subscribes to `/franka_controller/target_cartesian_velocity_percent` topic for target cartesian velocity input.

#### Launch Files:
- **`franka.launch.py`**: Launches the Franka robot with the controller.
- **`joint_impedance_ik_controller.launch.py`**: Launches the Joint Impedance controller.

### 2. `spacemouse_publisher`
This package provides a ROS 2 node that reads input from a 3Dconnexion SpaceMouse and publishes it as `geometry_msgs/Twist` messages.

#### Key Features:
- Publishes SpaceMouse input to the `/franka_controller/target_cartesian_velocity_percent` topic.
- Uses the `pyspacemouse` library for device communication.

#### Launch Files:
- **`spacemouse_publisher.launch.py`**: Launches the SpaceMouse publisher node.

### 3. `gripper_manager`
This package provides a ROS 2 node for managing the gripper of the Franka robot. It allows sending commands to control the gripper's width and perform homing actions.

#### Key Features:
- Subscribes to `/gripper_client/target_gripper_width_percent` for gripper width commands.
- Supports homing and move actions for the gripper.

#### Launch Files:
- **`franka_gripper_client.launch.py`**: Launches the gripper manager node.

## VS-Code Dev-Container

We recommend using the provided VS Code Dev Container for a consistent development environment with all dependencies pre-installed.

To get started, open the project in VS Code so that the `.devcontainer` folder is part of the workspace. When prompted, select "Reopen in Container." If not prompted, open the Command Palette (`Ctrl+Shift+P`) and run "Dev Containers: Reopen in Container."

The initial container build may take a few minutes. Once complete, you'll be inside a ready-to-use development environment. For more details, see the [VS-Code Dev-Containers documentation](https://code.visualstudio.com/docs/devcontainers/containers).


If you choose not to use the Dev-Container, please refer to the [Local Setup](#local-setup) section below for manual installation instructions.

## Local Setup

### Prerequisites

- **ROS 2 Humble Desktop** must be installed.  
  See the [official installation guide](https://docs.ros.org/en/humble/Installation.html) for instructions.
- **libfranka** and **franka_ros2** must be installed.  
  Refer to the [Franka Robotics documentation](https://frankarobotics.github.io/docs/index.html) for installation steps and compatibility information.

> 💡 **Hint:**  
> You can also find example installation commands for `libfranka` and `franka_ros2` in the [Dockerfile](./.devcontainer/Dockerfile) located in the `.devcontainer` directory. These commands can be copy-pasted for your local setup.

### Further Dependency Installations

After installing the prerequisites, you may need to install additional dependencies required by this workspace. For this you can run the `install_workspace_dependencies.bash` script.

If you add new dependencies to your packages, remember to update the relevant `requirements.txt`, `requirements_dev.txt` or `package.xml` files and re-run the script.


## Build and Test

### Building the Project

To build the project, use the following `colcon` command with CMake arguments, required for clang-tidy:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=ON
```

### Testing the Project

The project comes with a set of tests, which can be executed using the following command:

```bash
colcon test
```

## Getting Started

To get started with the SpaceMouse publisher and Joint Impedance controller:

### Identify the Connected SpaceMouse Devices (OPTIONAL) 

This step is only required if multiple SpaceMouses are connected to the device. To determine which connected HID (Human Interface Device) corresponds to each SpaceMouse, run the following command:
```bash
grep -H . /sys/class/hidraw/hidraw*/device/uevent | grep SpaceMouse
```

This will output something like: `/sys/class/hidraw/hidraw1/device/uevent:HID_NAME=3Dconnexion SpaceMouse Wireless BT` 

In this example, **hidraw1** is the identifier for the SpaceMouse. Based on this, the `device_path` for this SpaceMouse would be: `/dev/hidraw1`

### Run the SpaceMouse Publisher 

Create a configuration file in `src/spacemouse_publisher/config/` or modify one of the provided example configuration files. Then launch the SpaceMouse publisher node to read input from the SpaceMouse and publish it as ROS 2 messages:

```bash
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py [config_file:=your_config.yaml]
```

The `config_file` argument is **optional**. If not provided, it defaults to `example_fr3_config.yaml` in the `spacemouse_publisher/config/` directory.

**Configuration parameters:**

- `namespace`: ROS 2 namespace (must match the robot and the gripper).
- `operator_position_front`:  
- Set `operator_position_front: True` if the operator is sitting in front of the robot.  
- Set `operator_position_front: False` if the operator is positioned elsewhere.  
*Hint: This aligns the coordinate system of the SpaceMouse to the end-effector. Rotating the SpaceMouse can achieve the same effect if, for example, you are seated on the robot's right side.*
- `device_path`: If multiple SpaceMouse devices are connected, you can specify the path as described in [Identify the Connected SpaceMouse Devices (OPTIONAL)](#identify-the-connected-spacemouse-devices-optional). If it is defined as `''`, the first detected device will be used.


### Launch the Joint Impedance Controller

Create a configuration file in `src/franka_arm_controllers/config/` or modify one of the provided example configuration files. Then launch the controller to send torque commands to the Franka robot:

```bash
   ros2 launch franka_arm_controllers joint_impedance_ik_controller.launch.py [robot_config_file:=your_config.yaml]
```

The `robot_config_file` argument is **optional**. If not provided, it defaults to `example_fr3_config.yaml` in the `franka_arm_controllers/config/` directory.

**Configuration parameters:**
- Most parameters are documented in [`franka.launch.py`](src/franka_arm_controllers/launch/franka.launch.py) (see line 16 and following).
- The `arm_mounting_orientation` parameter specifies the robot's mounting angles.  
  For details, see the section: [Arbitrary Mounting of the Robots (Experimental Feature)](#arbitrary-mounting-of-the-robots-experimental-feature).

### Launch the Gripper Manager

Create a configuration file in `src/gripper_manager/config/` or modify one of the provided example configuration files. Then launch the gripper manager node to control the gripper:

```bash
ros2 launch gripper_manager franka_gripper_client.launch.py [config_file:=your_config.yaml]
```

**Configuration parameters:**

- `namespace`: ROS 2 namespace (must match the robot and SpaceMouse publisher).
- **ROS 2 topic names** (customize as needed for your setup):
  - `grasp_action_topic`: (default: `/franka_gripper/grasp`)
  - `homing_action_topic`: (default: `/franka_gripper/homing`)
  - `gripper_command_topic`: (default: `/gripper_client/target_gripper_width_percent`)
  - `joint_states_topic`: (default: `/franka_gripper/joint_states`)
- `gripper_epsilon_inner`: Maximum allowed deviation for inner gripper width (default: `0.08`)
- `gripper_epsilon_outer`: Maximum allowed deviation for outer gripper width (default: `0.08`)
- `gripper_speed`: Maximum speed for gripper movement (default: `1.0`)
- `gripper_force`: Maximum force applied by the gripper (default: `70.0`)


## Arbitrary Mounting of the Robots (Experimental Feature)

The `arbitrary mounting` mode allows for custom, non-standard mounting of the robot via the Desk API (standard mounting: table-top). This feature is **experimental** and intended for pilot users only. We are actively working toward formal certification for arbitrary mounting. This feature will become officially supported once this process is complete.

---

> ⚠️ **Warning:** Mounting the robot at an orientation other than table-top voids the certification and the warranty of FR3. 
> It is intended for advanced users who understand the implications of bypassing safety mechanisms.
> Use at your own risk. 


### Conditions for Access

- This feature is **experimental** and currently **not certified** for general use.
- Customers must **sign a waiver** acknowledging the loss of:
  - All **safety-related functions**
  - All **warranty coverage**
- After receiving the signed waiver, we will enable the `arbitrary mounting` API endpoint for your account.

### How it works

When this mode is active:

- Control inputs from the **SpaceMouse** are interpreted in the **world coordinate system** instead of the robot’s base coordinate system.
- The robot’s **TCP (Tool Center Point)** will move in the **world direction** corresponding to the SpaceMouse input — regardless of how the robot is mounted.
- **Example:** Pushing the SpaceMouse forward moves the tool *"forward in the workspace,"* even if the robot is mounted upside-down or sideways.

This enables intuitive teleoperation in flexible or unconventional setups.

> ⚠️ **Note:** The control behavior described here is only relevant **if the `arbitrary mounting` feature is enabled and correctly configured for the robot**.
>
> The controller in this project **does not perform gravity compensation** itself — it only transforms control commands from the robot coordinate system into the world coordinate system.
>
> If the `arbitrary mounting` feature is **not properly enabled**, the robot will behave incorrectly or unsafely due to missing gravity compensation.
>
> Make sure the robot is configured for arbitrary mounting at the system level **before using this mode**.

### Configuration

To ensure correct operation with arbitrary mounting, **use the same mounting angles in your ROS configuration as you used when setting up arbitrary mounting at the system level**.

1. **Edit the Configuration File**  
   Open your config file in `franka_arm_controllers/config` and copy the arbitrary mounting angles (roll, pitch, yaw) from your system-level setup into the corresponding field`.

2. **Rebuild and Restart**  
   After saving your changes, rebuild your workspace and restart the controller for the new configuration to take effect.


## Known Issues & Troubleshooting

- **SpaceMouse Not Detected in Dev Container**  
  If the SpaceMouse publisher fails to detect the device while running in a development container, try restarting the container. 

