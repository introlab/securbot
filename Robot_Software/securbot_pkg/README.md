
# SecurBot Package

This package contains the navigation files for the different robotic platforms.

## Content

The package contains the following folders:

* config: Navigation configuration files (1 folder per base)
  * base_local_planner_params.yaml: Parameters for the local planner (TrajectoryPlannerROS) and the global planner (NavfnROS) of move_base
  * costmap_common_params.yaml: Parameters for the local and global costmap with the obstacle sources
  * global_costmap_params.yaml: Parameters for the global costmap
  * local_costmap_params.yaml: Parameters for the local costmap
  * smoother.yaml: Parameters for the yocs_velocity_smoother

* launch: ROS launch files
  * localization.launch: Starts the robot base and RTAB-Map in localization mode.
  * mapping.launch: Starts the robot base and RTAB-Map in mapping mode.
  * mapping.rviz: RViz config save file with the correct display to monitor mapping.
  * navigation.launch: Starts move_base node with the params from the config folder.
  * navigation.rviz: RViz config save file with the correct display to monitor navigation.
  * patrolExecutive.launch: Starts the ROS node that can execute a patrol on the robot.
  * teleop.launch: Starts the teleop node.
  * include: Contains the specific launch files for the hardware setup of each robot.
    * bases: Launch files for the specific base of the robot
    * frames: Frames of reference for the sensors
    * sensors: Lidar and camera launch files
    * securbot_bringup.launch: Starts the robot with the appropriate base, frames and sensors.
    * rtabmap_bringup.launch: Starts RTAB-Map in mapping or localization according to the provided arguments.
    * virtualDevices.launch: Start the map image generator and create the virtual cameras trough V4L2 loopback that will be opened by EasyRTC.
  * template_launch_files: Contains templates for creating new launch files.
  * demo:
    * securbot_demo.launch: Start the full SecurBot demo with RTAB-Map in mapping mode. It allows teleoperation and patrol trough the web interface.

* scripts: Python scripts for the waypoint navigation
* test: Test scripts for the package's scripts based on the ROS test framework and Python unittest.

## Usage

Visit [the wiki](https://github.com/introlab/securbot/wiki/ROS-Getting-Started) for examples on how to use these files.
