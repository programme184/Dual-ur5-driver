## Dual robots driver

## Requirements

Installation of **[Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) and **
    [universal_robot](https://github.com/ros-industrial/universal_robot)

### Usage

```
roslanch dual_robot_startup.launch
python dual_ur_control.py	# example of control joint 6
```
### Example
```
def shake_robot(robot, shake_delta):
    joint_degrees[5] +=  shake_delta
    # print('joint_degrees', joint_degrees)
    joint_positions = deg2rad(joint_degrees)
    print('desired rads', joint_positions)
    print('current rads', robot.get_joints())
    
    # use ik_fast to get the best solution
    ik_joints = ik_algorithm.ik_fast(joint_positions)
    print('ik_joints:\n', ik_joints)
    # robot.publisher(joint_positions)
    robot.move_joints(ik_joints , duration=0.1)
    
    # rospy.sleep(3)  # Sleep for 0.5 seconds between movements
    joint_degrees = rad2deg(robot.get_joints())
    print('after',joint_degrees[5])
```
Modify above code block to set ur5 joint positions.

### Reference

Control ur5 real hardware without MoveIt!:


https://github.com/rmqlife/ros_ur5

Inverse kinematics algorithm


https://github.com/cambel/ur_ikfast
