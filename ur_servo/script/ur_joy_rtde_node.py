#!/usr/bin/env python2

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
from copy import deepcopy
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState


class state_jointctl():
    def __init__(self, UR_IP_ADDRESS, SERVER_PORT):
        self.ur_jointctl = RTDEControl(UR_IP_ADDRESS, -1, RTDEControl.FLAG_USE_EXT_UR_CAP, SERVER_PORT)
        # self.ur_jointctl.zeroFtSensor()

    def set_TargetJoint(self, target_joint_q, is_Noblocked):
        return self.ur_jointctl.moveJ(target_joint_q, 0.1, 0.8, is_Noblocked)

    def set_TargetTCP(self, target_pose, is_Noblocked):
        return self.ur_jointctl.moveL(target_pose, 0.1, 0.8, is_Noblocked)


class state_receive():
    def __init__(self, UR_IP_ADDRESS):
        variables = ["timestamp", "actual_q", "actual_TCP_pose", "actual_TCP_force"]
        self.ur_receive = RTDEReceive(UR_IP_ADDRESS, 125, variables, True, False, -1)

    def get_ActuralJointPose(self):
        # Actual joint angles in rad
        return self.ur_receive.getActualQ()

    def get_ActuralTCPPose(self):
        # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz)
        # where rx, ry and rz is a rotation vector representation of 
        # the tool orientation
        return self.ur_receive.getActualTCPPose()
    
    def get_FtRawWrench(self):
        return self.ur_receive.getFtRawWrench()

    def get_ActualTCPForce(self):
        return self.ur_receive.getActualTCPForce()

class RTDE_node:
    def __init__(self, UR_IP_ADDRESS, UR_ID, SERVER_PORT):
        self.UR_ID = UR_ID
        self.tf_listener = tf.TransformListener()
        self.prev_time = rospy.Time.now()
        self.sub_servo = rospy.Subscriber("/pose_servo_cmd", PoseStamped, self.pose_servo_cb)
        self.ur_pose_state_pub = rospy.Publisher('/ur_pose_state', PoseStamped, queue_size=1)
        self.ur_joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.ur_wrench_state_pub = rospy.Publisher('/ur_wrench_state', Wrench, queue_size=1)
        print("[{}] Starting state_receive ...".format(self.UR_ID))
        self.urReceive = state_receive(UR_IP_ADDRESS)
        print("[{}] Starting state_jointctl ...".format(self.UR_ID))
        self.urJointctl = state_jointctl(UR_IP_ADDRESS, SERVER_PORT)
        print("[{}] Finished state_jointctl".format(self.UR_ID))
        self.Pose_initial = [0.4, -0.15, 0.3, -2.22144, 2.22144, 0.0]
        
        self.is_stop_it_now = False
        rospy.Timer(rospy.Duration(0, 1e8), self.publish_pose_state) # 10 Hz


    def publish_pose_state(self, event):
        """
        publish ur pose
        """
        pose_to_send = PoseStamped()
        pose_to_send.header.frame_id = "base2end"
        pose_to_send.header.stamp = rospy.Time.now()
        current_joints = self.urReceive.get_ActuralJointPose()

        ### force msg
        # current_wrench = self.urReceive.get_FtRawWrench() # Only for > polyscope 5.9
        # rospy.loginfo("current_wrench: {}".format(current_wrench))
        current_tcpforce = self.urReceive.get_ActualTCPForce()
        wrench_to_send = Wrench()
        wrench_to_send.force.x = current_tcpforce[0]
        wrench_to_send.force.y = current_tcpforce[1]
        wrench_to_send.force.z = current_tcpforce[2]
        wrench_to_send.torque.x = current_tcpforce[3]
        wrench_to_send.torque.y = current_tcpforce[4]
        wrench_to_send.torque.z = current_tcpforce[5]
        self.ur_wrench_state_pub.publish(wrench_to_send)
        # rospy.loginfo("current_tcpforce:{}".format(current_tcpforce))

        ## 6 Dof pose msg
        current_pose = self.urReceive.get_ActuralTCPPose()
        pose_to_send.pose.position.x = current_pose[0]
        pose_to_send.pose.position.y = current_pose[1]
        pose_to_send.pose.position.z = current_pose[2]
        # direct axis-angle rotation 
        pose_to_send.pose.orientation.x = current_pose[3]
        pose_to_send.pose.orientation.y = current_pose[4]
        pose_to_send.pose.orientation.z = current_pose[5]
        pose_to_send.pose.orientation.w = 1
        self.ur_pose_state_pub.publish(pose_to_send)

        ## 6 joints
        current_joints_state = JointState()
        current_joints_state.header.frame_id = ''
        current_joints_state.header.stamp = pose_to_send.header.stamp
        current_joints_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        current_joints_state.position = current_joints
        self.ur_joint_state_pub.publish(current_joints_state)


    def pose_servo_cb(self, msg):
        Pose_delta = msg
        Head_frame_id = Pose_delta.header.frame_id
        identity, frame_id = Head_frame_id[:3], Head_frame_id[4:] # Get option
        if identity == "COM": print(Pose_delta)
        if frame_id == "STOP":
            self.is_stop_it_now = True
            return
        elif frame_id == "APOSE": # absolute pose
            Pose_curr = self.urReceive.get_ActuralTCPPose()
            new_pose, pose_bias = self.computePoseFromAbsolutePose(Pose_curr, Pose_delta)
            if (pose_bias > 1e-03 and pose_bias < 1.5):
                self.target_pose = new_pose
                self.pose_bias = pose_bias
                self.overwrite_servo = False
                self.control_mode = 'Pose'
                rospy.loginfo("[{}] {} Update Pose bias: {:.04f}. \New XYZ: {:.02f} {:.02f} {:.02f},RxRyRz:{:.02f} {:.02f} {:.02f}".format(
                    self.UR_ID, identity, pose_bias, new_pose[0], new_pose[1], new_pose[2], \
                    new_pose[3], new_pose[4], new_pose[5]))
            else: rospy.logwarn("[{}] ERROR distance bias for pose cnt: {}".format(self.UR_ID, pose_bias))

        elif frame_id == "DPOSE": # delta pose: only valid in position, not orientation for convenience ctl.
            Pose_curr = self.urReceive.get_ActuralTCPPose()
            # print('current Pose_ur:', Pose_curr)
            new_pose, pose_bias = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
            if (pose_bias > 1e-03 and pose_bias < 1.5):
                self.target_pose = new_pose
                self.pose_bias = pose_bias
                self.overwrite_servo = False
                self.control_mode = 'Pose'
                rospy.loginfo("[{}] {} Update Pose bias: {:.04f}. \nNew XYZ: {:.02f} {:.02f} {:.02f},RxRyRz:{:.02f} {:.02f} {:.02f}".format(
                    self.UR_ID, identity, pose_bias, new_pose[0], new_pose[1], new_pose[2], \
                    new_pose[3], new_pose[4], new_pose[5]))
            else: rospy.logwarn("[{}] ERROR distance bias for pose cnt: {}".format(self.UR_ID, pose_bias))

        elif frame_id == "JOINT": # delta joint
            Joint_curr = self.urReceive.get_ActuralJointPose()
            # print('current Pose_ur:', Pose_curr)
            new_joint, joint_bias = self.computeJointFromDeltaJoint(Joint_curr, Pose_delta)
            if (joint_bias > 1e-03 and joint_bias < 2.0):
                self.target_joint = new_joint
                self.joint_bias = joint_bias
                self.overwrite_servo = False
                self.control_mode = 'Joint'
                rospy.loginfo("[{}] {} Update joint bias: {:.04f}. Current Joint 123,456: {:.02f} {:.02f} {:.02f}, {:.02f} {:.02f} {:.02f}".format(
                    self.UR_ID, identity, joint_bias, Joint_curr[0], Joint_curr[1], Joint_curr[2],Joint_curr[3], \
                    Joint_curr[4], Joint_curr[5]))
            else: rospy.logwarn("[{}] ERROR joint bias for pose cnt: {}".format(joint_bias))

        elif frame_id == "SERVO": # delta pose servo
            Pose_curr = self.urReceive.get_ActuralTCPPose()
            # print('current Pose_ur:', Pose_curr)
            new_pose, pose_bias = self.computePoseFromDeltaPose(Pose_curr, Pose_delta)
            if (pose_bias > 1e-03 and pose_bias < 2e-02):
                self.target_pose = new_pose
                self.pose_bias = pose_bias
                self.overwrite_servo = True
                self.control_mode = 'Servo'
                rospy.loginfo("[{}] {} Update SERVO bias: {:.04f}. \nCurrent XYZ: {:.04f} {:.04f} {:.04f},RxRyRz:{:.04f} {:.04f} {:.04f}".format(
                    self.UR_ID, identity, pose_bias, Pose_curr[0], Pose_curr[1], Pose_curr[2], \
                    Pose_curr[3], Pose_curr[4], Pose_curr[5]))
            elif (identity != "JOY"): rospy.logwarn("[{}] ERROR pose bias for pose cnt: {}".format(self.UR_ID, pose_bias))

    @staticmethod
    def computePoseFromDeltaPose(Pose_curr, Pose_delta):
        # roll, pitch, yaw = tf.transformations.euler_from_quaternion([
        #     Pose_delta.pose.orientation.x, Pose_delta.pose.orientation.y,
        #     Pose_delta.pose.orientation.z, Pose_delta.pose.orientation.w])
        roll, pitch, yaw = 0., 0., 0.
        
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
    def computePoseFromAbsolutePose(Pose_curr, Pose_abs):
        # roll, pitch, yaw = tf.transformations.euler_from_quaternion([
        #     Pose_abs.pose.orientation.x, Pose_abs.pose.orientation.y,
        #     Pose_abs.pose.orientation.z, Pose_abs.pose.orientation.w])
        
        # direct roll pitch yaw
        new_pose = deepcopy(Pose_curr)
        new_pose[0] = Pose_abs.pose.position.x
        new_pose[1] = Pose_abs.pose.position.y
        new_pose[2] = Pose_abs.pose.position.z
        # new_pose[3] = roll
        # new_pose[4] = pitch
        # new_pose[5] = yaw
        new_pose[3] = Pose_abs.pose.orientation.x
        new_pose[4] = Pose_abs.pose.orientation.y
        new_pose[5] = Pose_abs.pose.orientation.z
        bias = 0
        len(new_pose)
        for i in range(3): bias += (Pose_curr[i] - new_pose[i]) ** 2 
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
                    rospy.logwarn("[{}] Joystick call stop by pressing SELECT.".format(self.UR_ID))
                    rospy.signal_shutdown("[{}] Joystick call stop by pressing SELECT.".format(self.UR_ID))

                vel_pose, acc, lookahead_time, gain, vel_joint = 0.1, 0.1, dt*2, 200, 0.4

                servo_target_pose = self.target_pose
                pose_delta = self.pose_bias
                servo_target_joint = self.target_joint
                joint_delta = self.joint_bias
                
                if self.control_mode == 'Pose': # Motion mode for long distance
                    # Get time restricts the speed to higher
                    block_time = pose_delta / vel_pose
                    if block_time < 0.2: block_time = 0.2
                    rospy.logwarn("[{}] Try to reach posedetla {} in time {} with vel {}".format(self.UR_ID, pose_delta, block_time, vel_pose))
                    t_start = self.urJointctl.ur_jointctl.initPeriod()
                    self.urJointctl.ur_jointctl.servoL(servo_target_pose, vel_pose, acc, block_time, lookahead_time, gain)
                    self.urJointctl.ur_jointctl.waitPeriod(t_start)
                    self.control_mode = 'Servo'
                    self.overwrite_servo = False

                elif self.control_mode == 'Joint':
                    block_time = joint_delta / vel_joint
                    if block_time < 0.2: block_time = 0.2
                    rospy.logwarn("[{}] Try to reach jointdetla {} in time {} with vel {}".format(self.UR_ID, joint_delta, block_time, vel_joint))
                    print('[{}] servo_target_joint:'.format(self.UR_ID), servo_target_joint)
                    self.urJointctl.ur_jointctl.servoJ(servo_target_joint, vel_joint, acc, block_time, lookahead_time, gain)
                    self.control_mode = 'Servo'
                    self.overwrite_servo = False

                elif self.control_mode == 'Servo' and self.overwrite_servo: # Servo mode for minor increment
                    t_start = self.urJointctl.ur_jointctl.initPeriod()
                    self.urJointctl.ur_jointctl.servoL(servo_target_pose, vel_pose, acc, dt, lookahead_time, gain)
                    self.urJointctl.ur_jointctl.waitPeriod(t_start)
                    self.overwrite_servo = False

        except KeyboardInterrupt:
            print("[{}] Control Interrupted!".format(self.UR_ID))
            self.urJointctl.ur_jointctl.servoStop()
            self.urJointctl.ur_jointctl.stopScript()


def main():
    import sys
    UR_ID, UR_IP_ADDRESS, SERVER_PORT = sys.argv[1], sys.argv[2], int(sys.argv[3])

    rospy.init_node("ur_rdte_node")
    rospy.loginfo("[{}] Initialized node with IP :{}".format(UR_ID, UR_IP_ADDRESS, SERVER_PORT))

    app = RTDE_node(UR_IP_ADDRESS, UR_ID, SERVER_PORT)
    
    rospy.loginfo("[{}] ROS spinning and UR realtiming...".format(UR_ID))
    rospy.loginfo("[{}] Play joystick or publish posetopic will work...".format(UR_ID))
    app.servoPose(dt=0.05)
    # rospy.spin()

if __name__ == "__main__":
    main()
