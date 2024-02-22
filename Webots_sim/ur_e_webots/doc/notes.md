## Start up

```
roslaunch ur_e_webots ur5e.launch

python3 single_ur5_contrl.py	# control robot by joints
```

## Tutorial

reference:
https://github.com/ros2/ros2_documentation/blob/rolling/source/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.rst

https://docs.ros.org/en/rolling/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html

https://docs.ros.org/en/galactic/Tutorials/Advanced/Simulators/Webots.html

https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html

https://cyberbotics.com/doc/reference/ros-api

## Debug

```
catkin_install_python(PROGRAMS scripts/universal_robots_ros.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

cancle out above code

### Test subcriber

```
initial:
position: [-9.541538056176294e-05, 5.717749411861837e-05, 1.5863537579404305e-05, -6.530287060435569e-06, 4.1101365557516475e-07, -4.114185676227323e-07]

after changed
position: [-4.7180412489871475e-05, -3.3705970447810385e-05, -1.2988805106221652e-05, 5.061829141515903e-06, -2.714868409616212e-06, 1.8627468122474513e-07]

```
