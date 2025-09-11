#!/usr/bin/env python3
import rospy, numpy as np
from sensor_msgs.msg import Imu

def normalize(msg):
    # 归一化加速度：1 g
    acc = np.array([msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z])
    acc_norm = np.linalg.norm(acc)
    if acc_norm > 1e-6:
        acc = acc / acc_norm          # 1 g
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc

    # 角速度保持 rad/s
    msg.angular_velocity_covariance = [0.01]*9
    msg.linear_acceleration_covariance = [0.01]*9
    pub.publish(msg)

rospy.init_node('normalize_imu')
sub = rospy.Subscriber('/livox/imu', Imu, normalize)
pub = rospy.Publisher('/livox/imu_raw', Imu, queue_size=500)
rospy.spin()