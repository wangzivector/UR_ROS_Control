# Introduction
THis repository just mofify some congfigurations to ease the adoptation pose tracking in UR robot.
Features include **how to use URsim to simulate and configure the config.yaml file for default ur_bringup.launch**, and **modify the joy topic and make it compactible with joy node,** using joystick instead of spacenav to control the robot. 
Further information and guides can be found in `../Command.md` and `ur_servo/README.md`. 

# Usage
## Joystick to twist

### **Configure controllers setting**
A must of moveit_servo is to configure the robot controller to `jointGroupPositionController`. 
Details check the `ur_servo/README.md`. or the followings:


> #### **Switch controller**
> use the following command to configure after launch `ur5_bringup.launch`
>``
>rosrun rqt_controller_manager rqt_controller_manager --force-discover
>``
>
>Controller loading and starting configuration is paramed in ur5_bringup.sh. For convinence, it can be modified to used as different application. 
>`scaled_pos_joint_traj_controller` (something like this) are usually used in `ur5_moveit_planning_execution.launch`. However, `joint_group_position_controller` maybe used in moveit_servo package when adopting twist control and pose tracking application. Try to check them before pose excution.
>Note: for example, using `JointGroupPositionController`, modifiy the `ur5e_bringup.launch` -> controllers setting is prefered. also remeber to ckeck the `ur_x_controllers.yaml` file if there are the related controller parameters.

### **Run ur_bringup.launch**
`roslaunch ur_gazebo ur5e_bringup.launch` or `roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=127.0.0.1`. 

### **Run moveit_excution.launch**
setup move group configuration:
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch  limited:=true
```

### **Run joystick_toTwist.launch**
```
roslaunch moveit_servo joystick_toTwist.launch
```

Then use a joystick to test the command is ok. 


## Pose Tracking
This example startup is similar to the [Joystick to twist](#joystick-to-twist). Just change the last launch file to `roslaunch moveit_servo pose_tracking_example.launch`. The pose tracking will auto control the UR pose to move in circle (along y / z axis).

Note: You may be want to modify it to subscribe a PoseStamped msg to externally control the robot, just do it. But remeber to debug and run it safely.  