#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

# 重力加速度，单位 m/s²
G = 9.7936

# 定义发布者
converted_imu_pub = None

def imu_callback(imu_msg):
    global converted_imu_pub
    # 复制原始IMU消息，避免直接修改传入的消息
    converted_imu = imu_msg

    # 将加速度从 g 转换为 m/s²
    converted_imu.linear_acceleration.x *= G
    converted_imu.linear_acceleration.y *= G
    converted_imu.linear_acceleration.z *= G

    # 打印转换后的加速度数据
    rospy.loginfo("Converted Acceleration (N/kg): x=%.4f, y=%.4f, z=%.4f",
                  converted_imu.linear_acceleration.x,
                  converted_imu.linear_acceleration.y,
                  converted_imu.linear_acceleration.z)

    # 发布转换后的IMU数据
    converted_imu_pub.publish(converted_imu)

def imu_subscriber():
    global converted_imu_pub
    # 初始化ROS节点
    rospy.init_node('imu_converter', anonymous=True)

    # 创建发布者，发布转换后的IMU数据到新话题
    converted_imu_pub = rospy.Publisher('/livox/imu_c', Imu, queue_size=10)

    # 订阅 /livox/imu 话题
    rospy.Subscriber('/livox/imu', Imu, imu_callback)

    # 进入循环等待消息
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_subscriber()
    except rospy.ROSInterruptException:
        pass