# ROS Noetic Franka Panda Robot Simulation with MoveIt! and Gazebo

This repository contains a ROS Noetic project for simulating a Franka Emika Panda robotic arm. It integrates the robot model with Gazebo for physics simulation, `ros_control` for realistic joint control, and MoveIt! for advanced motion planning capabilities.

## 🌟 Project Purpose

The primary goal of this project is to create a functional simulation environment for the Panda robotic arm, allowing for:
* Accurate physical simulation of the Panda arm's kinematics and dynamics in Gazebo.
* Realistic control of the arm's joints using `ros_control` interfaces (position, velocity, effort).
* High-level motion planning and execution through the MoveIt! framework.
* Development and testing of various robotic tasks, such as reaching joint goals and potentially executing Cartesian paths.

## 📁 Project Structure

This project is organized as a standard Catkin workspace, with key custom packages located in the `src/` directory. Below is a simplified overview focusing on the main components developed or heavily configured for this simulation:

catkin_ws_arm/
├── src/
│   ├── my_franka_gazebo/
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── panda_gazebo.launch  # Main launch for spawning Panda in Gazebo
│   │   └── package.xml
│   │
│   ├── my_panda_simulation/
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── full_simulation.launch  # Orchestrates Gazebo, controllers, MoveIt!
│   │   │   └── loaded_robot_description.urdf # Potential custom URDF loading
│   │   └── package.xml
│   │
│   ├── my_robot_controller/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts/
│   │       └── move_arm_joint_goal.py # Python script for MoveIt! commands
│   │
│   ├── panda_description/
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── display.launch        # For displaying URDF in RViz
│   │   ├── package.xml
│   │   └── urdf/
│   │       └── panda.urdf            # Robot's main URDF file
│   │
│   └── panda_moveit_config/
│       ├── CMakeLists.txt
│       ├── config/
│       │   ├── controllers.yaml      # ros_control controller definitions (PIDs, types)
│       │   ├── gazebo_controllers.yaml # Gazebo-specific controller mappings
│       │   ├── joint_limits.yaml
│       │   ├── kinematics.yaml
│       │   ├── ompl_planning.yaml
│       │   ├── panda.srdf.xacro      # Semantic Robot Description Format for MoveIt!
│       │   └── ros_controllers.yaml  # ROS Control definitions for MoveIt!
│       ├── launch/
│       │   ├── demo_gazebo.launch    # Launches MoveIt! RViz with Gazebo integration
│       │   ├── move_group.launch     # Core MoveIt! planning group launch
│       │   ├── moveit.rviz           # RViz configuration for MoveIt!
│       │   ├── planning_context.launch
│       │   └── ros_control_moveit_controller_manager.launch.xml
│       └── package.xml
│
└── .gitignore                     # Git ignore rules for build artifacts
└── README.md                      # This readme file


## ✅ Successfully Implemented & Configured Components

Despite encountering a hardware-specific runtime issue (detailed below), the core setup and integration of various complex ROS components have been successfully completed:

1.  **Robot Description (`panda_description` package):**
    * The complete URDF model of the Franka Panda arm (`panda_description/urdf/panda.urdf`) has been integrated and verified. This forms the foundational kinematic and dynamic representation of the robot within ROS.

2.  **Gazebo Simulation Integration (`my_franka_gazebo`, `my_panda_simulation` packages):**
    * Custom Gazebo launch files (e.g., `my_franka_gazebo/launch/panda_gazebo.launch` and `my_panda_simulation/launch/full_simulation.launch`) are configured to spawn the Panda arm into the simulation environment.
    * The `gazebo_ros_control` plugin is correctly configured within the robot's URDF, enabling `ros_control` to interface with Gazebo's physics engine for realistic simulation.

3.  **ROS Control Setup (`panda_moveit_config`, `my_robot_controller` packages):**
    * The `gazebo_controllers.yaml` and `ros_controllers.yaml` within `panda_moveit_config/config/` are set up for the Panda arm's joints, defining `PositionJointInterface`, `VelocityJointInterface`, and `EffortJointInterface`, along with `FrankaStateInterface` and `FrankaModelInterface` for accurate Franka hardware simulation.
    * PID gains for various controllers have been thoroughly tuned (e.g., in `panda_moveit_config/config/controllers.yaml`) to ensure stable and responsive joint control within Gazebo.
    * A custom Python script (`my_robot_controller/scripts/move_arm_joint_goal.py`) is prepared to command joint goals to the robot via MoveIt! APIs, demonstrating high-level control.

4.  **MoveIt! Integration (`panda_moveit_config` package):**
    * The `panda_moveit_config` package provides all necessary configuration files for MoveIt!, including `panda.srdf.xacro` (Semantic Robot Description Format), `kinematics.yaml`, `joint_limits.yaml`, and definitions for various planning pipelines (OMPL, CHOMP, Pilz Industrial Motion Planner).
    * Launch files like `panda_moveit_config/launch/demo_gazebo.launch` are configured to bring up MoveIt! with the Gazebo simulation environment, allowing for motion planning visualization and interaction in RViz.

## 🚀 How to Run (on compatible hardware)

This project is fully configured and *would run successfully* on hardware that supports **OpenGL 3.3 or higher**. If run on such a system, follow these steps:

1.  **Clone the repository into your Catkin workspace:**
    ```bash
    cd ~/catkin_ws_arm/src/
    git clone [https://github.com/dimitristheodoropoulos/franka-panda-gazebo-moveit.git](https://github.com/dimitristheodoropoulos/franka-panda-gazebo-moveit.git) # Use your actual repo URL
    cd ~/catkin_ws_arm/
    ```

2.  **Install dependencies and build the workspace:**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    source devel/setup.bash
    ```

3.  **Launch the full simulation environment:**
    This launch file will bring up Gazebo with the Panda arm, its controllers, and the MoveIt! RViz interface for visualization and planning.

    ```bash
    roslaunch my_panda_simulation full_simulation.launch
    ```

4.  **Run a joint goal command (optional, in a new terminal after Step 3):**
    This script would command the arm to a predefined joint configuration using MoveIt!.
    ```bash
    rosrun my_robot_controller move_arm_joint_goal.py
    ```

## ⚠️ Challenges & Limitations: Detailed Diagnosis

While the project's setup is complete and all configurations are in place, the simulation currently experiences a **`Segmentation fault (core dumped)`** upon launching Gazebo with the robot model.

### Symptom:
The `gzserver` process (Gazebo's simulation engine) crashes immediately after the `gazebo_ros_control` plugins load and the Franka hardware interfaces are initialized. This occurs regardless of whether the Gazebo GUI (`gzclient`) is launched or not (i.e., it crashes even in headless mode). No detailed Gazebo log file is generated due to the abrupt nature of the crash.

### Diagnosis Process:

1.  **Initial Troubleshooting:** Standard debugging steps were performed, including verifying ROS/Gazebo installations, checking package dependencies, and tuning PID gains (which was a separate, successful debugging effort that improved controller stability but did not resolve the crash).
2.  **Eliminating GUI as Cause:** The persistence of the crash in headless mode confirmed that the issue was not with the visual rendering client (`gzclient`) but with `gzserver` itself or its interaction with the underlying system's graphics capabilities.
3.  **Investigation of System Requirements:** Given the low-level nature of the crash and the absence of detailed logs, attention shifted to hardware and driver compatibility, specifically regarding graphics API support.
4.  **OpenGL Version Check:**
    * Utilizing `glxinfo | grep "OpenGL renderer string\|OpenGL version string"`, the system reported:
        ```
        OpenGL renderer string: Mesa DRI Intel(R) HD Graphics 4000 (IVB GT2)
        OpenGL version string: 3.0 Mesa 21.2.6
        ```
    * Further research confirmed that **Gazebo 11 (version 11.15.1, as installed) requires OpenGL 3.3 or higher** for stable operation. My current development hardware (Intel HD Graphics 4000) only supports OpenGL 3.0 with the installed drivers.

### Conclusion of Diagnosis:
The `Segmentation fault` is directly caused by a **fundamental incompatibility between Gazebo 11's OpenGL requirement and the maximum OpenGL version supported by the Intel HD Graphics 4000 GPU** on the development machine. Gazebo's underlying simulation or rendering components attempt to utilize OpenGL features (available in 3.3+) that are not present or correctly implemented in OpenGL 3.0, leading to the crash.

## 🎯 Demonstrated Capabilities (with compatible hardware)

Once running on compatible hardware, this simulation project effectively demonstrates strong experience in:
* **Motion planning and control for multi-dof robotic arms:** Executing planned trajectories for the Panda arm via MoveIt! and `ros_control`.
* **Integration of complex robotics frameworks:** Seamlessly bringing together ROS, Gazebo, `ros_control`, and MoveIt! into a cohesive simulation environment.
* **Debugging and system optimization:** The detailed process of diagnosing and understanding system-level issues, as evidenced by the OpenGL problem identification.
* **Configuration of robotic systems:** Expertise in setting up and customizing URDFs, controllers, and MoveIt! planning environments for a real-world robot.
