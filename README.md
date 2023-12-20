# UR ROS Pose Control
This repository collects several ways to control the pose of UR robots, including pose tracking. The method are divided by python-based package `/ur_servo` and cpp-based package `/moveit_servo`, which have their own dependencies. Most packages/nodes are related [MoveIt!](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) but run in various independent interface. Besides, [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html) are implemented in some nodes `/ur_servo` to control UR with/without ROS. In short, `/ur_servo` and `/moveit_servo` (as well as nodes inside) are independent in build and implementation. More likely to let you check and see the ways to do what you want with UR. 

## Key features 
- Well-structured control python script for UR-E series
- **Multiple UR robots** supported (at least 3 URs tested)
- Real-time incremental pose control
- Work with or without ROS
> All these features can be identified inside `ur_servo/launch/ur_rtde.launch`, `ur_servo/launch/ur_rtde_multiple.launch`, and `ur_servo/UR_control_rtde/ur_rtde.py`.


# Features
## ur_servo
* Examples of using python [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html) independently control UR, see: [ur_rtde_circle.py](ur_servo/UR_control_rtde/ur_rtde_circle.py) and [ur_rtde.py](ur_servo/UR_control_rtde/ur_rtde.py).

* ROS wrapper of ur_rtde to control pose of UR, see: [ur_joy_rtde_node.py](ur_servo/script/ur_joy_rtde_node.py).

* Example of using python `moveit_commander` to control UR pose, see: [ur_joy_rtde.py](ur_servo/script/ur_joy_rtde_node.py).

* Complicated method using python `moveit_ros_planning_interface` package to move to a specific pose, see: [joy_moveit_inter.py](ur_servo/script/joy_moveit_inter.py).

* Most nodes supports Joystick control, joy msg wrapper see: [joy_delta_pose.py](ur_servo/script/joy_delta_pose.py).

## moveit_servo
> Mainly modify the configuration to work with URsim, UR real robot and gazebo. It is confusing to make it work in UR, if you are also struggling, feel free to check the configuration in (`ur_servo/config`), launch files in (`ur_servo/launch`) as well as the [`ur_robot_driver_configuraion.zip`]() which share the configuration of `controllers.yaml` settings and `ur_bringup.launch` using in ur_robot_dirver package.

* Use joystick to control twist of UR TCP, check: [joystick_toTwist.launch](moveit_servo/launch/joystick_toTwist.launch) or [joystick_to_twist.cpp](moveit_servo/src/teleop_examples/joystick_to_twist.cpp).

* Modify the official moveit pose tracking example works in UR robot. see: [pose_tracking_example.launch](moveit_servo/launch/pose_tracking_example.launch) and its [config file](moveit_servo/config/ur_simulated_config.yaml).

# Usage

The instruction of each node are in `README.md` inside packages: [moveit_servo/README.md](moveit_servo/README.md) and [ur_servo/README.md](ur_servo/README.md). Please check if you need. However, the dependency are not given, please check the source and install them yourself.

Some shell commands when running nodes are also shared in [Command.md](Command.md). 