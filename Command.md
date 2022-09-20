# Just some commands

### ROS driver with URsim UR5
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=127.0.0.1

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true

roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```
### Some commond
* bring up real ur5e contorl node 
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.113.130.112 kinematics_config:=$(rospack find ur_robot_driver)/config/e11_ur5e_robot_calibration.yaml
```
* control Arm with joint position
```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover
```
* publish FT sensor
```
roslaunch onrobot_hex_ft_sensor demo.launch
```
* load controller manager
```
rosrun rqt_controller_manager rqt_controller_manager --force-discover
```
* joystick
```
roslaunch gripper_server joystick_control_publish.launch 
```

### rvizVisual and control ur5e 
* bring up real ur5e
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.113.130.112 limited:=true kinematics_config:=$(rospack find ur_robot_driver)/config/e11_ur5e_robot_calibration.yaml
```
* moveit config
```
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch limited:=true
```
* rviz vis
```
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true
```

### simulate ur5e in Gazebo
* load gazebo
```
roslaunch ur_gazebo ur5e_bringup.launch
```
* moveit config
```
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=true
```
* rviz vis
```
roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz
```
### Simulate ur5e with joystick
```
roslaunch ur_gazebo ur5e_bringup.launch

roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=true limited:=true

roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz

roslaunch ur_servo control_moveit_intface.launch # ros moveit interface
or
rosrun ur_servo pymoveit_contr.py # ros moveit commander
```
