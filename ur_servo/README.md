# Introduction
This repository is setuped for controlling pose of UR end-effector using **python** scripts. There are three python packages (`RTDE`, `moveit_ros_planning_interface` and `moveit_commander`) adopted independently to achieve the same goal: **Move UR's TCP to desired poses fast**. (**If you don't want to take MoveIt, `script/ur_joy_rtde.py` with excellent servo rate(20-100Hz) will satisify most of your needs.**)  

## RTDE-based scripts

* `UR_control_rtde/ur_rtde_circle.py` gives example only using UR python package [RTDE](https://sdurobotics.gitlab.io/ur_rtde/) to control UR pose **without** ROS, MoveIt! or the external_control URCap. 
  
* `UR_control_rtde/ur_rtde.py` is also presented for reference of UR I/O Pin control with RTDE.  

* `script/ur_joy_rtde.py` gives the ros wrapping node to subscribe a pose msg to control the UR pose with RTDE. 


## MoveIt!-based scripts
* `script/ur_joy_rtde.py` share an example of using `moveit_commander` which is introduced in official tutorial [MoveIt python API usage](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html). Basically, the script merely uses command `moveit_commander.MoveGroupCommander(group_name).go(desired_pose)` to move UR arm to desired poses. NOTE: when use rtde, donot open external control is ok.

* `joy_moveit_inter.py` is a more complicated method using `moveit_ros_planning_interface` package to move to a specific pose. The script accepts pose increment(by joystick commands) and add them to current pose. This method is much slower than others because the planning process and exceution are separated as individual trigger signals.

# Usage
## script/ur_joy_rtde.py
1. If you want to use a joystick to control the UR robot, run the joystick related launch first: `roslaunch ur_servo joystick_control_publish.launch`, which will convert joy commands to pose changes and send to the `ur_joy_rtde.py`.
2. If you want to control the UR by externally publishing a `PoseStamped` msg, check the class `RTDE_node` and refer to the callback function `sample_code_of_pose_servo_CB(self, msg)` to subscribe the related topic.
3. Configure the `UR_IP_ADDRESS` inside the script, then run it. OR just run:
```
rosrun ur_servo ur_joy_rtde_node.py _ip:=10.113.130.112
```
NOTE: the script is default using the external_control.URCap, after run the script you should start the external_control.URCap; or your just remove the flag `RTDEControl.FLAG_USE_EXT_UR_CAP` at:
```
    self.ur_jointctl = RTDEControl(UR_IP_ADDRESS, -1, RTDEControl.FLAG_USE_EXT_UR_CAP)
```
**It is really annoying that always some errors pop up when using the RTDE, check its [issue page](https://gitlab.com/sdurobotics/ur_rtde/-/issues/) maybe helpful.**

Please check other package or tools available to control UR. Here are some reference for you:

[python-urx](https://github.com/SintefManufacturing/python-urx) : much more elegent way to control UR. using Real-time port (500 Hz) with port 30003. 

[REMOTE CONTROL VIA TCP/IP](https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/) : some basic stuff of UR communication.



## script/moveit_commander_contr.py
1. Connect robot and check ip.
2. Start `ur5_bringup.launch`
3. Use `rosrun rqt_controller_manager rqt_controller_manager --force-discover`, open the `scaled_pos_joint_traj_controller`
4. Start moveit excecution `ur5_moveit_planning_execution.launch`
5. run `rosrun ur_servo  moveit_commander_contr.py`
6. If you want to use a joystick to control the UR robot, run the joystick related launch first: `roslaunch ur_servo joystick_control_publish.launch`, which will convert joy commands to pose changes and send to the `ur_joy_rtde.py`.
7. If you want to control the UR by externally publishing a `PoseStamped` msg, check the class `MoveGroupPythonIntefaceTutorial` and refer to the callback function `sample_code_of_pose_servo_CB(self, msg)` to subscribe the related topic.

## joy_moveit_inter.py
0. Run `ur5_bringup.launch` and `ur5_moveit_planning_execution.launch` like [last step](#scriptmoveitcommandercontrpy).
1. Run `roslaunch ur5e_moveit_config moveit_rviz.launch`. Then click the **External Commun** button on rviz.
2. Run the ros node by: `rosrun ur_servo joy_moveit_inter.py`
3. If always waiting informing 'Unable to initialize planning group' likewise, check the following topic: `rostopic hz /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full`, and try to drag the eef maker inside the RVIZ. after the mentioned topic is published, rosnode will work and listen to `/joy` (check it if necessary too) topic and is able to move the marker using joystick.   

## joystick_control_publish.launch
1. `joystick_control_publish.launch` launch file run the ros standard Joy package node (`/joy` msg) and process the button/axis values to pose increment (`/joy_delta_pose` msg). 
2. Four axis on the left and L1/L2 /R1/R2 are used to control six dof of UR TCP, SELECT button send a STOP signal via head.frame_id to shutdown rosnode.

# Problems
### **ur_rtde package installation maybe quite tricky** 

If you want to use it in python3, just run `pip3 install ur_rtde` can be fine. However, if willing to implemented in python2 (e.g. ROS 1), `pip install ur_rtde` may go wrong because dependency of pybind11. A robust way is to complie and build install the source on your own following [rtde installation guide](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html). **NOTE: DO REMEMBER** to set the git option (`git submodule update --init --recursive`) and cmake command (`cmake -DPYBIND11_PYTHON_VERSION=2.x ..`) as memetioned in [rtde installation guide](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html).


### **URsim and its IP**

URsim installation and setup can be disaster. Not because of the steps but the comfusing errors popped up. Here are some key steps to ease you and your ubuntu, I mean it:
1. Download URsim package in UR official website in linux version 
2. If ubuntu 18, install JAVA8 with apt: `sudo apt install openjdk-8-jre-headless`
3. Switch JAVA version to JAVA 8: `sudo update-alternatives --config java`
4. Follow this [repo.](https://github.com/arunavanag591/ursim) to modify the `./install.sh` (just change `openjdk-6-jre` to `openjdk-8-jre`)
5. REMOVE the `libcurl3` word inside `./install.sh` in the ursim file
6. sudo run: `sudo bash ./install.sh`
7. cd ursim/.. and change permission of all ursim files: `sudo chmod -R 777 /ursim-xxx`
8. Run (do not use sudo) the start shell: `./start-ursim.sh`

Using URsim and serve it as real robot is ok when using moveit. Note that the robot_ip setting is 127.0.0.1. 
An unexpected problem when using the external_control URCap and ur5_bring_up.sh: if move the robot using URsim pannel, reclick the stop/run button to restart the external_control URCap to establish connection with ur5_bring_up.sh.

### **Switch controller**

``
rosrun rqt_controller_manager rqt_controller_manager --force-discover
``
Controller loading and starting configuration is paramed in ur5_bringup.sh. For convinence, it can be modified to used as different application. 
`scaled_pos_joint_traj_controller` (something like this) are usually used in `ur5_moveit_planning_execution.launch`. However, `joint_group_position_controller` maybe used in moveit_servo package when adopting twist control and pose tracking application. Try to check them before pose excution.
Note: for example, using `JointGroupPositionController`, modifiy the `ur5e_bringup.launch` -> controllers setting is prefered. also remeber to ckeck the `ur_x_controllers.yaml` file if there are the related controller parameters.

---
**Licence**: No idea what it means yet. Forgive me :)
