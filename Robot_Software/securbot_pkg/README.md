# SecurBot Package
This package contains the navigation files for the different robotic platforms.

## Content
The package contains the following folders:

* config: Navigation configuration files (1 folder per robot)
   * base_local_planner_params.yaml: Parameters for the local planner (TrajectoryPlannerROS) and the global planner (NavfnROS) of move_base
   * costmap_common_params.yaml: Parameters for the local and global costmap with the obstacle sources
   * global_costmap_params.yaml: Parameters for the global costmap
   * global_costmap_params.yaml: Parameters for the local costmap
   * smoother.yaml: Parameters for the yocs_velocity_smoother
* launch: ROS launch files
   * localization.launch: Starts the robot base and RTAB-Map in localization mode.
   * mapping.launch: Starts the robot base and RTAB-Map in mapping mode.
   * mapping.rviz: Starts rviz with the required display for mapping.
   * navigation.launch: Starts move_base node with the params from the config folder.
   * navigation.rviz: Starts rviz with the required display for navigation.
   * robot_bringup.launch: Starts the robot with the appropriate base, frames and sensors.
   * securbot_demo.launch:
   * securbot_demo_teleop.launch:
   * securbot_demo_teleop_part1.launch:
   * securbot_demo_teleop_part2.launch:
   * securbot_demo_turtlebot.launch:
   * teleop.launch:
   * virtualDevices.launch:
   * virtualKinect.launch:
   * virtualMap.launch:
   * waypointDecoder.launch:
   * include: Contains the specific launch files for the hardware setup of each robot.
      * bases: Launch files for the specific base of the robot
      * frames: Frames of reference for the sensors
      * sensors: Lidar and camera launch files
   * template_launch_files: Contains templates for creating new launch files.

* scripts: Python scripts for the waypoint navigation
* test: Autonomous ROS testing files

## Commands

