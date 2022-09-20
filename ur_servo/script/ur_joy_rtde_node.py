#!/usr/bin/env python2

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
from copy import deepcopy
import rospy
import tf
from geometry_msgs.msg import PoseStamped


UR_IP_ADDRESS = "127.0.0.1"
if rospy.has_param("/ur_rdte_node/ip"):
    UR_IP_ADDRESS = rospy.get_param("/ur_rdte_node/ip")


class state_jointctl():
    def __init__(self):
        self.ur_jointctl = RTDEControl(UR_IP_ADDRESS, -1, RTDEControl.FLAG_USE_EXT_UR_CAP)

    def set_TargetJoint(self, target_joint_q, is_Noblocked):
        return self.ur_jointctl.moveJ(target_joint_q, 0.1, 0.1, is_Noblocked)

    def set_TargetTCP(self, target_pose, is_Noblocked):
        return self.ur_jointctl.moveL(target_pose, 0.1, 0.1, is_Noblocked)


class state_receive():
    def __init__(self):
        variables = ["timestamp", "actual_q", "actual_TCP_pose"]
        self.ur_receive = RTDEReceive(UR_IP_ADDRESS, 125, variables, True, False, -1)

    def get_ActuralTCPPose(self):
        # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz),
        # where rx, ry and rz is a rotation vector representation of 
        # the tool orientation
        return self.ur_receive.getActualTCPPose()

class RTDE_node:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.prev_time = rospy.Time.now()
        self.sub = rospy.Subscriber("/joy_delta_pose", PoseStamped, self.joy_delta_pose_CB, queue_size=1)
        # self.sub = rospy.Subscriber("/pose_to_servo", PoseStamped, self.sample_code_of_pose_servo_CB, queue_size=1)

        print("starting state_receive ...")
        self.urReceive = state_receive()
        print("starting state_jointctl ...")
        self.urJointctl = state_jointctl()
        print("Finished state_jointctl")
        self.Pose_initial = [-0.142, -0.427, 0.215, -0.0, -3.178, -0.0184]
        self.is_moving = False
    
    def sample_code_of_pose_servo_CB(self, msg):
        """
        This function is an example of how to subscribe a pose to send to UR robot 
        [NOT TESTED]
        """
        if self.is_moving:
            return
        ## received msg as posestamped()
        new_pose = msg # as a PoseStamped() object

        ## convert to a 6 dof array (x,y,z,rx,ry,rz)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            new_pose.pose.orientation.x, new_pose.pose.orientation.y,
            new_pose.pose.orientation.z, new_pose.pose.orientation.w])
            
        new_pose_array = [0, 0, 0, 0, 0, 0]
        new_pose_array[0] += new_pose.pose.position.x
        new_pose_array[1] += new_pose.pose.position.y
        new_pose_array[2] += new_pose.pose.position.z
        new_pose_array[3] += roll
        new_pose_array[4] += pitch
        new_pose_array[5] += yaw
        self.is_moving = True
        self.urJointctl.set_TargetTCP(new_pose_array, False)
        self.is_moving = False

    def joy_delta_pose_CB(self, msg):
        if self.is_moving:
            return

        Pose_delta = msg
        if Pose_delta.header.frame_id == "STOP":
            rospy.loginfo("Joystick call stop by pressing SELECT.")
            rospy.signal_shutdown("Joystick call stop by pressing SELECT.")
        Pose_curr = self.urReceive.get_ActuralTCPPose()
        print('current Pose_ur:', Pose_curr)
        new_pose, bias = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
        print("Pose bias: ", bias)
        if (bias > 1e-05):
            self.is_moving = True
            self.urJointctl.set_TargetTCP(new_pose, False)
            self.is_moving = False

    def computePoseFromDeltaPose(self, Pose_curr, Pose_delta):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            Pose_delta.pose.orientation.x, Pose_delta.pose.orientation.y,
            Pose_delta.pose.orientation.z, Pose_delta.pose.orientation.w])
        
        new_pose = deepcopy(Pose_curr)
        new_pose[0] += Pose_delta.pose.position.x
        new_pose[1] += Pose_delta.pose.position.y
        new_pose[2] += Pose_delta.pose.position.z
        new_pose[3] += roll
        new_pose[4] += pitch
        new_pose[5] += yaw
        bias = Pose_delta.pose.position.x ** 2 + Pose_delta.pose.position.y ** 2 + \
                Pose_delta.pose.position.z ** 2 + roll ** 2  + pitch ** 2  + yaw ** 2 
        return new_pose, bias

def main():

    rospy.init_node("ur_rdte_node")
    rospy.loginfo("Initialized node")
    rospy.logwarn("IP is %s" %  UR_IP_ADDRESS)
    app = RTDE_node()
    
    rospy.loginfo("ROS spinning...")
    rospy.spin()


if __name__ == "__main__":
    main()