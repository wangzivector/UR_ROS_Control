#!/usr/bin/env python
import sys
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from math import pi
import tf
from moveit_commander.conversions import pose_to_list
from copy import deepcopy

CONTROL_RATE = 10

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self, initial_pose = False):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:")
        print(robot.get_group_names())

        group_name = group_names[1]
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print ("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print ("============ End effector link: %s" % eef_link)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (robot.get_current_state())
        self.sub = rospy.Subscriber("/joy_delta_pose", PoseStamped, self.joy_delta_pose_CB, queue_size=1)
        # self.sub = rospy.Subscriber("/pose_to_servo", PoseStamped, self.sample_code_of_pose_servo_CB, queue_size=1)

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pre_time = rospy.get_rostime()

    def sample_code_of_pose_servo_CB(self, msg):
        """
        This function is an example of how to subscribe a pose to send to UR robot 
        [NOT TESTED]
        """
        move_group = self.move_group
        
        ## received msg as posestamped()
        new_pose = msg # as a PoseStamped() object
        Pose_curr = self.move_group.get_current_pose().pose

        print('======================= Goal pose :')
        print(new_pose)
        # move_group.set_pose_target(new_pose)

        move_group.allow_looking(True)
        plan = move_group.go(joints = new_pose, 
                    wait=True if(self.cal_pose_bias(Pose_curr, new_pose) > 1000) else False)

    def joy_delta_pose_CB(self, msg):
        if msg.header.frame_id == "STOP":
            rospy.loginfo("Joystick call stop by pressing SELECT.")
            rospy.signal_shutdown("Joystick call stop by pressing SELECT.")
        
        sec_steps = msg.header.stamp.to_sec() - self.pre_time.to_sec()
        if sec_steps < 1.0 / CONTROL_RATE:
            rospy.logwarn("Received too fast pose msg, Current control rate: %d" % CONTROL_RATE)
            return
        
        self.pre_time = msg.header.stamp
        Pose_delta = msg.pose
        Pose_curr = self.move_group.get_current_pose().pose
        print('======================= current pose : ')
        print(Pose_curr)
        new_pose = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
        
        close = self.go_to_pose_goal(Pose_curr, new_pose)
        if not close:
            rospy.ROSInterruptException

    def computePoseFromDeltaPose(self, Pose_curr, Pose_delta):
        new_pose = deepcopy(Pose_curr)

        new_q = tf.transformations.quaternion_multiply([
            Pose_curr.orientation.x, Pose_curr.orientation.y,
            Pose_curr.orientation.z, Pose_curr.orientation.w],
            [Pose_delta.orientation.x, Pose_delta.orientation.y,
            Pose_delta.orientation.z, Pose_delta.orientation.w])
        
        new_pose.position.x += Pose_delta.position.x
        new_pose.position.y += Pose_delta.position.y
        new_pose.position.z += Pose_delta.position.z
        new_pose.orientation.x = new_q[0]
        new_pose.orientation.y = new_q[1]
        new_pose.orientation.z = new_q[2]
        new_pose.orientation.w = new_q[3]
        return new_pose

    def go_to_pose_goal(self, Pose_curr, new_pose):
        move_group = self.move_group

        print('======================= Goal pose :')
        print(new_pose)
        # move_group.set_pose_target(new_pose)

        move_group.allow_looking(True)
        plan = move_group.go(joints = new_pose, 
                    wait=True if(self.cal_pose_bias(Pose_curr, new_pose) > 1000) else False)
        rospy.sleep(rospy.Duration(1.0/(CONTROL_RATE*1.5)))

        # Calling `stop()` ensures that there is no residual movement
        # move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        print('======================= Final pose : ')
        print(current_pose)
        return all_close(new_pose, current_pose, 0.01) and plan

    def cal_pose_bias(slef, pre_pose, new_pose):
        bias = 0
        bias += (pre_pose.orientation.x - new_pose.orientation.x)**2
        bias += (pre_pose.orientation.y - new_pose.orientation.y)**2
        bias += (pre_pose.orientation.z - new_pose.orientation.z)**2
        bias += (pre_pose.orientation.w - new_pose.orientation.w)**2
        bias += (pre_pose.position.x - new_pose.position.x)**2
        bias += (pre_pose.position.y - new_pose.position.y)**2
        bias += (pre_pose.position.z - new_pose.position.z)**2
        bias = np.sqrt(bias)
        return bias
        

def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
