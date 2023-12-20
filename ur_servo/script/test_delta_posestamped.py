#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf

rospy.init_node('pose_to_servo_send_action')
pub = rospy.Publisher('/pose_servo_cmd', PoseStamped, queue_size=1)

# COM__STOP  COM_JOINT  COM_POSE


for i in range(4):
    x, y, z, roll, pitch, yaw = 0.41, -0.15, 0.16, -2.38, 2.04, 0.03

    absolute_pose_to_send = PoseStamped()
    absolute_pose_to_send.header.frame_id = "COM_APOSE"
    absolute_pose_to_send.pose.position.x = x
    absolute_pose_to_send.pose.position.y = y
    absolute_pose_to_send.pose.position.z = z
    # diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # absolute_pose_to_send.pose.orientation.x = diff_q[0]
    # absolute_pose_to_send.pose.orientation.y = diff_q[1]
    # absolute_pose_to_send.pose.orientation.z = diff_q[2]
    # absolute_pose_to_send.pose.orientation.w = diff_q[3]
    
    # direct roll pitch yaw
    absolute_pose_to_send.pose.orientation.x = roll
    absolute_pose_to_send.pose.orientation.y = pitch
    absolute_pose_to_send.pose.orientation.z = yaw
    absolute_pose_to_send.pose.orientation.w = 1

    pub.publish(absolute_pose_to_send)
    print(absolute_pose_to_send)
    if not rospy.is_shutdown(): rospy.sleep(10)


# for i in range(4):
#     delta_pose_to_send = PoseStamped()
#     delta_pose_to_send.header.frame_id = "COM_POSE"
#     delta_pose_to_send.pose.position.x = 0.1 if i % 2 == 1 else - 0.1
#     delta_pose_to_send.pose.orientation.w = 1.0
#     pub.publish(delta_pose_to_send)
#     print(delta_pose_to_send)
#     if not rospy.is_shutdown(): rospy.sleep(10)

# for i in range(4):
#     delta_pose_to_send = PoseStamped()
#     delta_pose_to_send.header.frame_id = "COM_JOINT"
#     delta_pose_to_send.pose.orientation.z = 0.1 if i % 2 == 1 else - 0.1
#     pub.publish(delta_pose_to_send)
#     print(delta_pose_to_send)
#     if not rospy.is_shutdown(): rospy.sleep(10)

delta_pose_to_send = PoseStamped()
delta_pose_to_send.header.frame_id = "COM_STOP"
pub.publish(delta_pose_to_send)