#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry

class OdomFrameChanger:
    def __init__(self):
        # 初始化节点
        rospy.init_node('odom_frame_changer', anonymous=True)
        
        # 订阅原始odom话题
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # 发布修改后的odom话题
        self.odom_pub = rospy.Publisher('odom_modified', Odometry, queue_size=10)
        
        rospy.loginfo("Odom Frame Changer node started...")

    def odom_callback(self, msg):
        # 复制原始消息
        modified_odom = msg
        
        # 修改frame_id
        modified_odom.header.frame_id = "odom"
        modified_odom.child_frame_id = "car_odom"

        
        # 发布修改后的消息
        self.odom_pub.publish(modified_odom)

if __name__ == '__main__':
    try:
        node = OdomFrameChanger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass