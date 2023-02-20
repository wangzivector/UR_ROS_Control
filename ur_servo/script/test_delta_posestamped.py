#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('pose_to_servo_send_action')
pub = rospy.Publisher('/pose_to_servo', PoseStamped, queue_size=1)

# COM__STOP  COM_JOINT  COM_POSE

for i in range(4):
    delta_pose_to_send = PoseStamped()
    delta_pose_to_send.header.frame_id = "COM_POSE"
    delta_pose_to_send.pose.position.x = 0.1 if i % 2 == 1 else - 0.1
    delta_pose_to_send.pose.orientation.w = 1.0
    pub.publish(delta_pose_to_send)
    print(delta_pose_to_send)
    if not rospy.is_shutdown(): rospy.sleep(10)

for i in range(4):
    delta_pose_to_send = PoseStamped()
    delta_pose_to_send.header.frame_id = "COM_JOINT"
    delta_pose_to_send.pose.orientation.z = 0.1 if i % 2 == 1 else - 0.1
    pub.publish(delta_pose_to_send)
    print(delta_pose_to_send)
    if not rospy.is_shutdown(): rospy.sleep(10)

delta_pose_to_send = PoseStamped()
delta_pose_to_send.header.frame_id = "COM_STOP"
pub.publish(delta_pose_to_send)