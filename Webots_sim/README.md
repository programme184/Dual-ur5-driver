### Requirements

Installing webots and webots-ros

https://cyberbotics.com/doc/guide/installation-procedure

http://wiki.ros.org/webots_ros


### Usage

1. Load webots world file:

   ```
   src/webots/ur_e_webots/worlds/main.wbt

   ```
2. Connect ros and webots

```
	roslaunch ur5e.launch
```

3. Control ur5 by setting joint values

```
python single_ur5_control.py
```


### Tutorials

https://cyberbotics.com/doc/reference/robot?tab-language=python
