## Dual robots driver

## Requirements

Installation of **[Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) and **
    [universal_robot](https://github.com/ros-industrial/universal_robot)

### Usage

```
roslanch dual_robot_startup.launch
python dual_ur_control.py	# example of control joint 6
```


### Reference

Control ur5 real hardware without MoveIt!:


https://github.com/rmqlife/ros_ur5

Inverse kinematics algorithm


https://github.com/cambel/ur_ikfast
