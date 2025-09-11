#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from enum import Enum
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String

class DockingStage(Enum):
    APPROACH = 1
    FINAL_DOCKING = 2

class TwoStageDockingController:
    def __init__(self):
        rospy.init_node('two_stage_docking_controller')
        
        # 添加控制标志和状态
        self.is_running = False
        self.current_state = "IDLE"
        
        # 初始化PID控制器字典
        self.pids = {
            'x': self.create_pid(),
            'z': self.create_pid(),
            'theta': self.create_pid()
        }
        
        self.current_stage = DockingStage.APPROACH
        self.stage_params = self.load_stage_params()
        
        # 添加控制话题订阅
        self.control_sub = rospy.Subscriber('/dock/control_cmd', String, self.control_callback)
        
        self.init_tf()
        self.init_publishers()
        self.update_pid_params(self.current_stage)
        self.control_loop()

    def control_callback(self, msg):
        """处理控制命令"""
        command = msg.data.lower()
        if command == "start" and not self.is_running:
            self.is_running = True
            self.current_stage = DockingStage.APPROACH
            self.update_pid_params(self.current_stage)
            self.current_state = "RUNNING"
            rospy.loginfo("Docking started")
        elif command == "stop":
            self.is_running = False
            self.current_state = "IDLE"
            self.send_stop_command()
            rospy.loginfo("Docking stopped")
        
        self.publish_state()

    def publish_state(self):
        """发布当前状态"""
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

    def create_pid(self):
        """创建PID控制器结构体"""
        return {
            'kp': 0.0,
            'kd': 0.0,
            'max': 0.0,
            'integral': 0.0,
            'prev_error': 0.0,
            'first_run': True
        }

    def load_stage_params(self):
        """加载两阶段参数"""
        return {
            DockingStage.APPROACH: {
                'target_distance': rospy.get_param("~approach_target_distance", 0.3),
                'pos_threshold': rospy.get_param("~approach_pos_threshold", 0.02),
                'angle_threshold': math.radians(rospy.get_param("~approach_angle_threshold", 5.0)),
                'max_linear': rospy.get_param("~approach_max_linear", 0.5),
                'max_angular': rospy.get_param("~approach_max_angular", 1.2),
                'pid_params': {
                    'x': {
                        'kp': rospy.get_param("~approach_x_kp", 0.4),
                        'kd': rospy.get_param("~approach_x_kd", 0.1),
                        'max': rospy.get_param("~approach_x_max", 1.0)
                    },
                    'z': {
                        'kp': rospy.get_param("~approach_z_kp", 0.5),
                        'kd': rospy.get_param("~approach_z_kd", 0.05),
                        'max': rospy.get_param("~approach_z_max", 0.6)
                    },
                    'theta': {
                        'kp': rospy.get_param("~approach_theta_kp", 0.6),
                        'kd': rospy.get_param("~approach_theta_kd", 0.2),
                        'max': rospy.get_param("~approach_theta_max", 1.0)
                    }
                }
            },
            DockingStage.FINAL_DOCKING: {
                'target_distance': rospy.get_param("~final_target_distance", 0.1),
                'pos_threshold': rospy.get_param("~final_pos_threshold", 0.005),
                'angle_threshold': math.radians(rospy.get_param("~final_angle_threshold", 1.5)),
                'max_linear': rospy.get_param("~final_max_linear", 0.2),
                'max_angular': rospy.get_param("~final_max_angular", 0.5),
                'pid_params': {
                    'x': {
                        'kp': rospy.get_param("~final_x_kp", 0.8),
                        'kd': rospy.get_param("~final_x_kd", 0.3),
                        'max': rospy.get_param("~final_x_max", 0.4)
                    },
                    'z': {
                        'kp': rospy.get_param("~final_z_kp", 0.3),
                        'kd': rospy.get_param("~final_z_kd", 0.1),
                        'max': rospy.get_param("~final_z_max", 0.3)
                    },
                    'theta': {
                        'kp': rospy.get_param("~final_theta_kp", 1.2),
                        'kd': rospy.get_param("~final_theta_kd", 0.5),
                        'max': rospy.get_param("~final_theta_max", 0.6)
                    }
                }
            }
        }

    def init_tf(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.prev_yaw = None

    def init_publishers(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/docking/state', String, queue_size=1)

    def update_pid_params(self, stage):
        """更新当前阶段的PID参数"""
        params = self.stage_params[stage]['pid_params']
        for axis in ['x', 'z', 'theta']:
            self.pids[axis]['kp'] = params[axis]['kp']
            self.pids[axis]['kd'] = params[axis]['kd']
            self.pids[axis]['max'] = params[axis]['max']
            # 重置控制器状态
            self.pids[axis]['integral'] = 0.0
            self.pids[axis]['prev_error'] = 0.0
            self.pids[axis]['first_run'] = True

    def compute_pid(self, axis, error, dt):
        """PID计算"""
        pid = self.pids[axis]
        if pid['first_run']:
            pid['prev_error'] = error
            pid['first_run'] = False
            return 0.0
        
        p_term = pid['kp'] * error
        d_term = pid['kd'] * (error - pid['prev_error']) / dt
        
        # 积分项（带抗饱和）
        pid['integral'] += error * dt
        i_term = 0.0  # 暂时禁用积分项
        
        output = p_term + i_term + d_term
        output = max(min(output, pid['max']), -pid['max'])
        
        pid['prev_error'] = error
        return output

    def check_stage_transition(self, z_error, yaw_error):
        current_params = self.stage_params[self.current_stage]
        if (abs(z_error) < current_params['pos_threshold'] and 
            abs(yaw_error) < current_params['angle_threshold']):
            if self.current_stage == DockingStage.APPROACH:
                rospy.loginfo("Transition to FINAL_DOCKING stage")
                self.current_stage = DockingStage.FINAL_DOCKING
                self.current_state = "FINAL_DOCKING"
                self.update_pid_params(self.current_stage)
                self.publish_state()
                return True
        return False

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def control_loop(self):
        rate = rospy.Rate(20)
        num=0
        while not rospy.is_shutdown():
            # 发布当前状态
            self.publish_state()
            
            # 如果未运行，继续等待
            if not self.is_running:
                rate.sleep()
                continue

            try:
                trans = self.tf_buffer.lookup_transform('tag_2', 'base', rospy.Time(0))
            except:
                rate.sleep()
                continue

            # 计算当前误差
            current_params = self.stage_params[self.current_stage]
            x_error = trans.transform.translation.x
            current_z = trans.transform.translation.z
            z_error = current_params['target_distance'] - current_z
            
            q = trans.transform.rotation
            _, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_error = self.normalize_angle(-pitch)

            # 计算控制量
            dt = 0.02  # 固定时间步长（50Hz）
            vx = self.compute_pid('z', z_error, dt)
            wz_x = self.compute_pid('x', x_error, dt)
            wz_theta = self.compute_pid('theta', yaw_error, dt)
            wz = 0.4 * wz_theta + 0.6 * wz_x

            # 速度限制
            vx = max(min(vx, current_params['max_linear']), -current_params['max_linear'])
            wz = max(min(wz, current_params['max_angular']), -current_params['max_angular'])

            # 发布指令
            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            # 阶段转换检查
            if self.current_stage == DockingStage.APPROACH:
                self.check_stage_transition(z_error, yaw_error)
            # 最终收敛检查
            if self.current_stage == DockingStage.FINAL_DOCKING:
                final_params = self.stage_params[DockingStage.FINAL_DOCKING]
                check1=(abs(z_error) < final_params['pos_threshold'] and
                    abs(x_error) < final_params['pos_threshold'] and
                    abs(yaw_error) < final_params['angle_threshold'])
                check2=(abs(z_error) < final_params['pos_threshold']*1.7 and
                    abs(x_error) < final_params['pos_threshold']*1.5 and
                    abs(yaw_error) < final_params['angle_threshold']*4)
                check3=False
                if check2:
                    if(abs(cmd.linear.x)<0.017):
                        print("num is ",num)
                        num=num+1
                        print("in here")
                        if(num>=100):
                            check3=True
                    else:
                        num=0
                # print(abs(z_error) ," ",final_params['pos_threshold']*1.7,
                #       abs(x_error) ," ", final_params['pos_threshold']*1.5,
                #       abs(yaw_error) ," ", final_params['angle_threshold']*1.6)
                print(abs(z_error) < final_params['pos_threshold']*1.7 ,"  " ,abs(x_error) < final_params['pos_threshold']*1.5, abs(yaw_error) < final_params['angle_threshold']*4,' ','num is ',num)
                if check1 or check3:
                    rospy.loginfo("Docking completed!")
                    self.current_state = "DOCKING_COMPLETED"
                    self.publish_state()
                    self.is_running = False
                    self.send_stop_command()
                    num=0
                    continue

            rate.sleep()

    def send_stop_command(self):
        cmd = Twist()
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        TwoStageDockingController()
    except rospy.ROSInterruptException:
        pass
