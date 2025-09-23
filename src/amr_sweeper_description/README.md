# AMR-Sweeper_description

This repository provides the full URDF (Unified Robot Description Format) files for the AMR-Sweeper robot, designed for autonomous cleaning operations. It includes 3D models, configuration files, and launch scripts to facilitate simulation and integration within a ROS 2 environment.



## Features
![structure](https://github.com/user-attachments/assets/c968ac2e-3a3e-4bcd-a6e1-c69000b22b92)

- URDF model of the AMR-Sweeper robot
- Simulation-ready launch files for Gazebo
- Detailed 3D meshes and configuration files
- Modular structure for easy customization

## Prerequisites

- ROS 2 (Humble)
- Gazebo (for simulation)


## Installation

### 1. Clone the Repository

Navigate to your ROS 2 workspace's `src` directory and clone the repository:


```bash
cd ~/ros2_ws/src
git clone https://github.com/O-Robotics/amr_sweeper_description.git
```


### 2. Install Dependencies

Ensure all dependencies are installed. If the repository specifies any ROS 2 packages in its `package.xml`, install them using:


```bash
rosdep install --from-paths src --ignore-src -r -y
```


### 3. Build the Package

Return to the root of your workspace and build the package:


```bash
cd ~/ros2_ws
colcon build --packages-select amr_sweeper_description
```


### 4. Source the Workspace

After building, source the setup script to overlay this package into your environment:


```bash
source install/setup.bash
```


## Usage

### Gazebo version adaptation instructions
If you only use RViz to load the model (not using Gazebo simulation), you can skip this step.


Currently it's for Gazebo sim. If you use traditional Gazebo (gazebo-classic, such as Gazebo 11), please modify the plugin section from:
```
<plugin 
            filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive" >
```
to:
```
<plugin
            filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">

```


### Launch the Robot Description

To launch the robot's description in RViz:


```bash
ros2 launch amr_sweeper_description rsp.launch.py
```


### Launch the Simulation

To launch the robot in a Gazebo simulation environment:


```bash
ros2 launch amr_sweeper_description amr-sweeper_description_sim.launch.py
```


If you want to see `/robot_description`, it's not a regular ROS topic, but a ROS parameter passed to the node:
```
ros2 param get /robot_state_publisher robot_description
```



## Folder Structure

The repository is organized as follows:

- `config/`
  - Contains configuration files for sensors, controllers, and other robot parameters.
- `launch/`
  - Python launch scripts to start the robot description and simulation environments.
- `meshes/`
  - 3D models (e.g., STL or DAE files) representing the robot's physical components.
- `setup/`
  - Scripts or files related to the initial setup or installation processes.
- `urdf/`
  - URDF files defining the robot's physical structure, joints, and links.
- `worlds/`
  - Gazebo world files for simulating the robot in various environments.
- `CMakeLists.txt`
  - Build instructions for CMake.
- `package.xml`
  - Package manifest file specifying dependencies and metadata.


## Acknowledgments

This package is developed and maintained by the O-Robotics team. For more information, visit our [Website](https://www.o-robotics.com/).
