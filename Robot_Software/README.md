# Robot Software
> This folder contains all the ROS modules created for the SecurBot project


## Get started:
To get started using the robot and its ROS modules, you need to have ROS kinetic install on your machine/computer and you need to create a symbolic link from of this folder in your workspace.


To start the robot, enter the following command :

`On Robot`
```
roslaunch turtlebot_bringup minimal.launch
roslaunch launchfiles/3dsensor.launch
```

` On Computer `
```
roslaunch rtabmap_ros demo_turtlebot_rviz.launch
```