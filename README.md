[IMU tools for ROS](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble#from-source-ros2)

[robot_localization](https://github.com/cra-ros-pkg/robot_localization)

[How to visualise IMU data coming from d435i?](https://github.com/IntelRealSense/realsense-ros/issues/2563)

[remap /imu/data_raw to other imu topic inside launch file](https://github.com/CCNYRoboticsLab/imu_tools/issues/142)

The ROS naming convention refers to ~name as a private name.
You can assign private parameters for a node directly from the command-line using a single underscore _ as a prefix.


[SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)

[ROS Quaternion to RPY](https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac)



### URDF 
Update your file my_robot.urdf and run it through the parser:4

```
$ check_urdf my_robot.urdf
```

That's it, you created your first URDF robot description! Now you can try to visualize the URDF using graphiz:5

```
$ urdf_to_graphiz my_robot.urdf
```

TODO:
Fix gazebo model bug.