#!/usr/bin/env python
import rospy
import roslib
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from sensor_msgs.msg import Joy
import tf


class JoyStatus:
    def __init__(self):
        self.center = False
        self.select = False
        self.start = False
        self.L3 = False
        self.R3 = False
        self.square = False
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.triangle = False
        self.cross = False
        self.circle = False
        self.L1 = False
        self.R1 = False
        self.L2 = False
        self.R2 = False
        self.left_analog_x = 0.0
        self.left_analog_y = 0.0
        self.right_analog_x = 0.0
        self.right_analog_y = 0.0


class PS3DualShockStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.cross = msg.buttons[0] == 1
        self.circle = msg.buttons[1] == 1
        self.triangle = msg.buttons[2] == 1
        self.square = msg.buttons[3] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1 
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.center = msg.buttons[10] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.up = msg.axes[5] > 0.5
        self.down = msg.axes[5] < -0.5
        self.left = msg.axes[4] > 0.5
        self.right = msg.axes[4] < -0.5
        self.orig_msg = msg


class StatusHistory:
    def __init__(self, max_length=10):
        self.max_length = max_length
        self.buffer = []

    def add(self, status):
        self.buffer.append(status)
        if len(self.buffer) > self.max_length:
            self.buffer = self.buffer[1 : self.max_length + 1]

    def all(self, proc):
        for status in self.buffer:
            if not proc(status):
                return False
        return True

    def latest(self):
        if len(self.buffer) > 0:
            return self.buffer[-1]
        else:
            return None

    def length(self):
        return len(self.buffer)

    def new(self, status, attr):
        """
        Detech key change to TRUE as new input as true and last is false

        :param _type_ status: _description_
        :param _type_ attr: _description_
        :return _type_: _description_
        """
        if len(self.buffer) == 0:
            return getattr(status, attr)
        else:
            return getattr(status, attr) and not getattr(self.latest(), attr)


class Joy_delta_pose_node:
    def __init__(self):
        self.frame_id = 'joy_delta_pose'
        self.history = StatusHistory(max_length=10)
        self.joy_pose_pub = rospy.Publisher("/joy_delta_pose", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

    def computePoseFromJoy(self, status):
        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.get_rostime()
        new_pose.header.frame_id = self.frame_id

        ####
        #### Pose XYZ
        ####
        scale = 200.0
        x_diff = (status.left_analog_y) / scale
        y_diff = (status.left_analog_x) / scale
        
        DTHETA = 0.005
        if status.L1:
            z_diff = DTHETA
        elif status.L2:
            z_diff = -DTHETA
        else:
            z_diff = 0.0

        ####
        #### Pose rXrYrZ
        ####
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.005

        scale = 200.0
        roll = (status.right_analog_y) / scale
        pitch = (status.right_analog_x) / scale

        if status.R1:
            if self.history.all(lambda s: s.R1):
                yaw = yaw + DTHETA * 2
            else:
                yaw = yaw + DTHETA
        elif status.R2:
            if self.history.all(lambda s: s.R2):
                yaw = yaw - DTHETA * 2
            else:
                yaw = yaw - DTHETA
         # increment of quaternion
        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        ####
        #### Joint 6
        ####
        DTHETA = 0.04
        joint_6 = 0.0
        if status.up:
            if self.history.all(lambda s: s.up):
                joint_6 = joint_6 + DTHETA * 2
            else:
                joint_6 = joint_6 + DTHETA
        elif status.down:
            if self.history.all(lambda s: s.down):
                joint_6 = joint_6 - DTHETA * 2
            else:
                joint_6 = joint_6 - DTHETA
        if status.right:
            if self.history.all(lambda s: s.right):
                joint_6 = joint_6 + DTHETA * 2
            else:
                joint_6 = joint_6 + DTHETA
        elif status.left:
            if self.history.all(lambda s: s.left):
                joint_6 = joint_6 - DTHETA * 2
            else:
                joint_6 = joint_6 - DTHETA

        #### Analyse mode
        if status.select:
            new_pose.header.frame_id = "JOY_STOP"
            return new_pose
        elif abs(joint_6) > 1e-2:
            new_pose.header.frame_id = "JOY_JOINT"
            new_pose.pose.orientation.z = joint_6
            return new_pose
        else:
            new_pose.header.frame_id = "JOY_SERVO"
            new_pose.pose.position.x = x_diff
            new_pose.pose.position.y = y_diff
            new_pose.pose.position.z = z_diff
            new_pose.pose.orientation.x = diff_q[0]
            new_pose.pose.orientation.y = diff_q[1]
            new_pose.pose.orientation.z = diff_q[2]
            new_pose.pose.orientation.w = diff_q[3]
            return new_pose

    def joyCB(self, msg):
        axes_amount = len(msg.axes)
        buttons_amount = len(msg.buttons)

        if axes_amount == 6 and buttons_amount == 12:
            status = PS3DualShockStatus(msg) # here
        else:
            raise Exception(
                "Unknown joystick, axes: {}, buttons: {}".format(
                    axes_amount, buttons_amount
                )
            )
        self.history.add(status)
        new_pose = self.computePoseFromJoy(status)
        self.joy_pose_pub.publish(new_pose)
        

def main():
    rospy.init_node("Joy_delta_pose_node")
    rospy.loginfo("Initialized node")
    app = Joy_delta_pose_node()
    
    rospy.loginfo("ROS spinning...")
    rospy.spin()


if __name__ == "__main__":
    main()