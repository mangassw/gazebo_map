#!/usr/bin/env python3

import rospy
from mbf_msgs.srv import CheckPath, CheckPathRequest
from mbf_msgs.srv import CheckPose, CheckPoseRequest
from std_srvs.srv import Empty

class CostmapClearer:
    def __init__(self):
        rospy.init_node('costmap_clearer_node')
        
        # 定义清除costmap的时间间隔（秒）
        self.clear_interval = 5.0
        
        # 等待服务可用
        rospy.loginfo("Waiting for MBF clear_costmap services...")
        
        try:
            # 等待所有costmap清除服务
            rospy.wait_for_service('/move_base_flex/clear_costmaps', timeout=10.0)
            self.clear_costmaps = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
            
            rospy.loginfo("All MBF services are available!")
        except rospy.ROSException as e:
            rospy.logerr("Service wait timeout: %s", str(e))
            rospy.signal_shutdown("Required services not available")
            return
            
        # 创建定时器，每隔clear_interval秒调用一次clear_all_costmaps函数
        self.timer = rospy.Timer(rospy.Duration(self.clear_interval), self.clear_all_costmaps)
        rospy.loginfo(f"Costmap clearer started, clearing every {self.clear_interval} seconds")

    def clear_all_costmaps(self, event):
        """清除所有costmap"""
        try:
            self.clear_costmaps()
            rospy.loginfo(f"All costmaps cleared at {rospy.get_time()}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        clearer = CostmapClearer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass