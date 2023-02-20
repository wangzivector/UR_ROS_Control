#!/usr/bin/env python2

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
from copy import deepcopy
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped


class state_jointctl():
    def __init__(self, UR_IP_ADDRESS):
        print("Try to open UR Cap on UR terminal, after the following command...")
        self.ur_jointctl = RTDEControl(UR_IP_ADDRESS, -1, RTDEControl.FLAG_USE_EXT_UR_CAP)

    def set_TargetJoint(self, target_joint_q, is_Noblocked):
        return self.ur_jointctl.moveJ(target_joint_q, 0.1, 0.8, is_Noblocked)

    def set_TargetTCP(self, target_pose, is_Noblocked):
        return self.ur_jointctl.moveL(target_pose, 0.1, 0.8, is_Noblocked)


class state_receive():
    def __init__(self, UR_IP_ADDRESS):
        variables = ["timestamp", "actual_q", "actual_TCP_pose"]
        self.ur_receive = RTDEReceive(UR_IP_ADDRESS, 125, variables, True, False, -1)

    def get_ActuralJointPose(self):
        # Actual joint angles in rad
        return self.ur_receive.getActualQ()

    def get_ActuralTCPPose(self):
        # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz)
        # where rx, ry and rz is a rotation vector representation of 
        # the tool orientation
        return self.ur_receive.getActualTCPPose()

class RTDE_node:
    def __init__(self, UR_IP_ADDRESS):
        self.tf_listener = tf.TransformListener()
        self.prev_time = rospy.Time.now()
        self.sub_joy = rospy.Subscriber("/joy_delta_pose", PoseStamped, self.pose_servo_CB, queue_size=1)
        self.sub_servo = rospy.Subscriber("/pose_to_servo", PoseStamped, self.pose_servo_CB, queue_size=1)

        print("Starting state_receive ...")
        self.urReceive = state_receive(UR_IP_ADDRESS)
        print("Starting state_jointctl ...")
        self.urJointctl = state_jointctl(UR_IP_ADDRESS)
        print("Finished state_jointctl")
        self.Pose_initial = [-0.142, -0.427, 0.215, -0.0, -3.178, -0.0184]
        self.is_stop_it_now = False

    def pose_servo_CB(self, msg):
        Pose_delta = msg
        Head_frame_id = Pose_delta.header.frame_id
        identity, frame_id = Head_frame_id[:3], Head_frame_id[4:] # Get option

        if identity == "COM": print(Pose_delta)

        if frame_id == "STOP":
            self.is_stop_it_now = True
            return

        elif frame_id == "POSE":
            Pose_curr = self.urReceive.get_ActuralTCPPose()
            # print('current Pose_ur:', Pose_curr)
            new_pose, pose_bias = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
            if (pose_bias > 1e-03 and pose_bias < 0.5):
                self.target_pose = new_pose
                self.pose_bias = pose_bias
                self.overwrite_servo = False
                self.control_mode = 'Pose'
                rospy.loginfo("{} Update Pose bias: {:.04f}. Current XYZ: {:.02f} {:.02f} {:.02f}".format(
                    identity, pose_bias, Pose_curr[0], Pose_curr[1], Pose_curr[2]))

        elif frame_id == "JOINT":
            Joint_curr = self.urReceive.get_ActuralJointPose()
            # print('current Pose_ur:', Pose_curr)
            new_joint, joint_bias = self.computeJointFromDeltaJoint(Joint_curr, Pose_delta)
            if (joint_bias > 1e-03 and joint_bias < 2e-01):
                self.target_joint = new_joint
                self.joint_bias = joint_bias
                self.overwrite_servo = False
                self.control_mode = 'Joint'
                rospy.loginfo("{} Update joint bias: {:.04f}. Current Joint 456: {:.02f} {:.02f} {:.02f}".format(
                    identity, joint_bias, Joint_curr[3], Joint_curr[4], Joint_curr[5]))
            return

        elif frame_id == "SERVO":
            Pose_curr = self.urReceive.get_ActuralTCPPose()
            # print('current Pose_ur:', Pose_curr)
            new_pose, pose_bias = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
            if (pose_bias > 1e-03 and pose_bias < 2e-02):
                self.target_pose = new_pose
                self.pose_bias = pose_bias
                self.overwrite_servo = True
                self.control_mode = 'Servo'
                rospy.loginfo("{} Update Servo bias: {:.04f}. Current XYZ: {:.02f} {:.02f} {:.02f}".format(
                    identity, pose_bias, Pose_curr[0], Pose_curr[1], Pose_curr[2]))

    @staticmethod
    def computePoseFromDeltaPose(Pose_curr, Pose_delta):
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
        return new_pose, math.sqrt(bias)

    @staticmethod
    def computeJointFromDeltaJoint(Joint_curr, Joint_delta):
        # we define position and orientation of xyz as joint 1-6
        j1, j2, j3, j4, j5, j6 = \
            Joint_delta.pose.position.x, Joint_delta.pose.position.y,\
            Joint_delta.pose.position.z, Joint_delta.pose.orientation.x,\
            Joint_delta.pose.orientation.y, Joint_delta.pose.orientation.z
        
        new_joint = deepcopy(Joint_curr)
        new_joint[0] += j1
        new_joint[1] += j2
        new_joint[2] += j3
        new_joint[3] += j4
        new_joint[4] += j5
        new_joint[5] += j6
        bias = abs(j1) + abs(j2) + abs(j3) + abs(j4) + abs(j5) + abs(j6) 
        return new_joint, bias
    
    def servoPose(self, dt=0.05):
        # Initial params
        self.target_pose = self.urReceive.get_ActuralTCPPose()
        self.pose_bias = 0
        self.target_joint = self.urReceive.get_ActuralJointPose()
        self.joint_bias = 0
        self.control_mode = 'Servo'
        self.overwrite_servo = False

        # loop_rate = rospy.Rate(1.0/dt/5)
        try:
            while not rospy.is_shutdown():
                # loop_rate.sleep()
                if self.is_stop_it_now:
                    self.urJointctl.ur_jointctl.servoStop(1.0)
                    rospy.logwarn("Joystick call stop by pressing SELECT.")
                    rospy.signal_shutdown("Joystick call stop by pressing SELECT.")

                vel_pose, acc, lookahead_time, gain, vel_joint = 0.03, 0.1, dt*2, 200, 0.2

                servo_target_pose = self.target_pose
                pose_delta = self.pose_bias
                servo_target_joint = self.target_joint
                joint_delta = self.joint_bias
                
                if self.control_mode == 'Pose': # Motion mode for long distance
                    # Get time restricts the speed to higher
                    block_time = pose_delta / vel_pose
                    if block_time < 0.2: block_time = 0.2
                    rospy.logwarn("Try to reach posedetla {} in time {} with vel {}".format(pose_delta, block_time, vel_pose))
                    self.urJointctl.ur_jointctl.servoL(servo_target_pose, vel_pose, acc, block_time, lookahead_time, gain)
                    self.control_mode = 'Servo'
                    self.overwrite_servo = False

                elif self.control_mode == 'Joint':
                    block_time = joint_delta / vel_joint
                    if block_time < 0.2: block_time = 0.2
                    rospy.logwarn("Try to reach jointdetla {} in time {} with vel {}".format(joint_delta, block_time, vel_joint))
                    self.urJointctl.ur_jointctl.servoJ(servo_target_joint, vel_joint, acc, block_time, lookahead_time, gain)
                    self.control_mode = 'Servo'
                    self.overwrite_servo = False

                elif self.control_mode == 'Servo' and self.overwrite_servo: # Servo mode for minor increment
                    t_start = self.urJointctl.ur_jointctl.initPeriod()
                    self.urJointctl.ur_jointctl.servoL(servo_target_pose, vel_pose, acc, dt, lookahead_time, gain)
                    self.urJointctl.ur_jointctl.waitPeriod(t_start)
                    self.overwrite_servo = False
        except KeyboardInterrupt:
            print("Control Interrupted!")
            self.urJointctl.ur_jointctl.servoStop()
            self.urJointctl.ur_jointctl.stopScript()


def main():
    UR_IP_ADDRESS = "127.0.0.1"
    if rospy.has_param("/ur_rdte_node/ip"):
        UR_IP_ADDRESS = rospy.get_param("/ur_rdte_node/ip")

    rospy.init_node("ur_rdte_node")
    rospy.loginfo("Initialized node")
    rospy.logwarn("IP is %s" %  UR_IP_ADDRESS)
    app = RTDE_node(UR_IP_ADDRESS)
    
    rospy.loginfo("ROS spinning and UR realtiming...")
    rospy.loginfo("Play joystick or publish posetopic will work...")
    app.servoPose(dt=0.05)
    # rospy.spin()

if __name__ == "__main__":
    main()
