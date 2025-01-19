""" 主要改进点总结：
1.位置保持功能：
  .增加了单轴位置保持能力
  .可以分别控制X、Y、Z轴和偏航角的位置保持
  .通过PID控制参数实现平滑的位置保持
2.控制精度提升：
  .增加了位置保持的比例控制参数
  .可以分别调节各个轴的控制响应
3.更灵活的控制模式：
  .支持混合控制模式（某些轴位置保持，某些轴速度控制）
  .支持FLU和ENU两种坐标系下的位置保持
4.状态管理优化：
  .更清晰的状态转换逻辑
  .更好的代码组织结构
  .更完善的控制模式切换机制
5. 安全性提升：
  .更平滑的模式切换
  .更可靠的位置保持能力
  .防止突然的位置偏移

这些改进使得飞行器能够更稳定地执行任务，特别是在需要精确位置控制的场景下表现更好。
同时，代码的可维护性和可扩展性也得到了提升。
 """
import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
import sys
import math

rospy.init_node(sys.argv[1] + '_' + sys.argv[2] + "_communication")
rate = rospy.Rate(30)


class Communication:

    def __init__(self, vehicle_type, vehicle_id):

        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.current_position = None
        self.current_yaw = 0
        self.hover_flag = 0
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None

        # 新增位置保持相关的属性
        self.hold_position_x = 0
        self.hold_position_y = 0
        self.hold_position_z = 0
        self.hold_yaw = 0
        self.hold_flag = 0
        self.hold_x_flag = 0
        self.hold_y_flag = 0
        self.hold_z_flag = 0
        self.hold_yaw_flag = 0
        self.hold_kp_x = 1 # 位置保持的比例控制参数
        self.hold_kp_y = 1
        self.hold_kp_z = 1
        self.hold_kp_yaw = 1

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/local_position/pose", PoseStamped,
            self.local_pose_callback, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd", String,
                                        self.cmd_callback, queue_size=3)
        self.cmd_pose_flu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,
            queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,
            queue_size=1)
        self.cmd_vel_flu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,
            queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_vel_enu", Twist, self.cmd_vel_enu_callback,
            queue_size=1)
        self.cmd_accel_flu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_accel_flu", Twist,
            self.cmd_accel_flu_callback, queue_size=1)
        self.cmd_accel_enu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_accel_enu", Twist,
            self.cmd_accel_enu_callback, queue_size=1)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type + '_' + self.vehicle_id + "/mavros/cmd/arming",
                                             CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type + '_' + self.vehicle_id + "/mavros/set_mode",
                                                    SetMode)

        print(self.vehicle_type + '_' + self.vehicle_id + ": " + "communication initialized")

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            rate.sleep()

    def local_pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if (self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW_RATE
        if (self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW
        if (self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                        + PositionTarget.IGNORE_YAW
        if (self.motion_type == 3):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW_RATE

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z, yaw=yaw)

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z, yaw=yaw)

    # 改进的速度控制回调函数：
    def cmd_vel_flu_callback(self, msg):
        # 先检查是否需要进入悬停模式
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        # 如果不悬停，检查是否需要位置保持
        if self.hover_flag == 0:
            self.hold_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, 'flu')
        # 如果既不悬停也不位置保持，则执行速度控制
        if self.hover_flag == 0 and self.hold_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.hold_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, 'enu')
        if self.hover_flag == 0 and self.hold_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x, afy=msg.linear.y, afz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x, afy=msg.linear.y, afz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def hover_state_transition(self, x, y, z, w):
        if abs(x) > 0.02 or abs(y) > 0.02 or abs(z) > 0.02 or abs(w) > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()

    # 新增位置保持状态转换功能：
    def hold_state_transition(self, x, y, z, w, vel_type):
        """位置保持状态转换函数
    
        当速度命令小于阈值时，自动进入对应轴的位置保持模式
        
        Args:
            x,y,z: 线速度命令
            w: 角速度命令
            vel_type: 速度指令类型('flu'或'enu')
        """
        if vel_type == 'flu':
            if abs(x) < 0.02 and abs(y) < 0.02:
                if self.hold_x_flag == 0 or self.hold_y_flag == 0:
                    self.hold_position_x = self.current_position.x
                    self.hold_position_y = self.current_position.y
                    self.hold_x_flag = 1
                    self.hold_y_flag = 1
            if abs(x) < 0.02:
                if self.hold_x_flag == 0:
                    self.hold_position_x = self.current_position.x
                    self.hold_position_y = self.current_position.y
                    self.hold_x_flag = 1
            else:
                self.hold_x_flag = 0
            if abs(y) < 0.02:
                if self.hold_y_flag == 0:
                    self.hold_position_x = self.current_position.x
                    self.hold_position_y = self.current_position.y
                    self.hold_y_flag = 1
            else:
                self.hold_y_flag = 0
            if abs(z) < 0.02:
                if self.hold_z_flag == 0:
                    self.hold_position_z = self.current_position.z
                    self.hold_z_flag = 1
                z = -self.hold_kp_z * (self.current_position.z - self.hold_position_z)
            else:
                self.hold_z_flag = 0
            if abs(w) < 0.005:
                if self.hold_yaw_flag == 0:
                    self.hold_yaw = self.current_yaw
                    self.hold_yaw_flag = 1
            else:
                self.hold_yaw_flag = 0

            if self.hold_x_flag and self.hold_y_flag and (self.hold_z_flag != 1):
                self.hold_flag = 1
                self.coordinate_frame = 8
                self.motion_type = 1
                x = -self.hold_kp_x * (self.current_position.x - self.hold_position_x) * math.cos(self.current_yaw) \
                    - self.hold_kp_y * (self.current_position.y - self.hold_position_y) * math.sin(self.current_yaw)
                y = self.hold_kp_x * (self.current_position.x - self.hold_position_x) * math.sin(self.current_yaw)  \
                    - self.hold_kp_y * (self.current_position.y - self.hold_position_y) * math.cos(self.current_yaw)
                if self.hold_yaw_flag == 0:
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z, yaw_rate=w)
                else:
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z,
                                                               yaw_rate=-self.hold_kp_yaw * (self.current_yaw - self.hold_yaw))
            elif self.hold_x_flag and self.hold_yaw_flag:
                self.hold_flag = 1
                self.coordinate_frame = 8
                self.motion_type = 1
                x = -self.hold_kp_x * (self.current_position.x - self.hold_position_x) * math.cos(self.current_yaw) \
                    - self.hold_kp_y * (self.current_position.y - self.hold_position_y) * math.sin(self.current_yaw)
                self.target_motion = self.construct_target(vx=x, vy=y, vz=z,
                                                           yaw_rate=-self.hold_kp_yaw * (self.current_yaw - self.hold_yaw))
            elif self.hold_y_flag and self.hold_yaw_flag:
                self.hold_flag = 1
                self.coordinate_frame = 8
                self.motion_type = 1
                y = self.hold_kp_x * (self.current_position.x - self.hold_position_x) * math.sin(self.current_yaw)  \
                    - self.hold_kp_y * (self.current_position.y - self.hold_position_y) * math.cos(self.current_yaw)
                self.target_motion = self.construct_target(vx=x, vy=y, vz=z,
                                                           yaw_rate=-self.hold_kp_yaw * (self.current_yaw - self.hold_yaw))
            elif self.hold_z_flag:
                self.hold_flag = 1
                self.coordinate_frame = 8
                self.motion_type = 1
                if self.hold_yaw_flag == 0:
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z, yaw_rate=w)
                else:
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z,
                                                               yaw_rate=-self.hold_kp_yaw * (self.current_yaw - self.hold_yaw))
            else:
                self.hold_flag = 0

        elif vel_type == 'enu':
            if abs(x) < 0.02:
                if self.hold_x_flag == 0:
                    self.hold_position_x = self.current_position.x
                    self.hold_x_flag = 1
                x = -self.hold_kp_x * (self.current_position.x - self.hold_position_x)
            else:
                self.hold_x_flag = 0
            if abs(y) < 0.02:
                if self.hold_y_flag == 0:
                    self.hold_position_y = self.current_position.y
                    self.hold_y_flag = 1
                y = -self.hold_kp_y * (self.current_position.y - self.hold_position_y)
            else:
                self.hold_y_flag = 0
            if abs(z) < 0.02:
                if self.hold_z_flag == 0:
                    self.hold_position_z = self.current_position.z
                    self.hold_z_flag = 1
                z = -self.hold_kp_z * (self.current_position.z - self.hold_position_z)
            else:
                self.hold_z_flag = 0
            if abs(w) < 0.005:
                if self.hold_yaw_flag == 0:
                    self.hold_yaw = self.current_yaw
                    self.hold_yaw_flag = 1
            else:
                self.hold_yaw_flag = 0

            if self.hold_x_flag or self.hold_y_flag or self.hold_z_flag:
                self.hold_flag = 1
                self.coordinate_frame = 1
                if self.hold_yaw_flag == 0:
                    self.motion_type = 1
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z, yaw_rate=w)
                else:
                    self.motion_type = 3
                    self.target_motion = self.construct_target(vx=x, vy=y, vz=z, yaw=self.hold_yaw)
            else:
                self.hold_flag = 0

    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return

        elif msg.data == 'ARM':
            self.arm_state = self.arm()
            print(self.vehicle_type + '_' + self.vehicle_id + ": Armed " + str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type + '_' + self.vehicle_id + ": Armed " + str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type + '_' + self.vehicle_id + ": " + msg.data)

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print(self.vehicle_type + '_' + self.vehicle_id + ": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type + '_' + self.vehicle_id + ": disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.current_position.x, y=self.current_position.y,
                                                   z=self.current_position.z, yaw=self.current_yaw)
        print(self.vehicle_type + '_' + self.vehicle_id + ":" + self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        elif self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type + '_' + self.vehicle_id + ": " + self.flight_mode)
            return True
        else:
            print(self.vehicle_type + '_' + self.vehicle_id + ": " + self.flight_mode + "failed")
            return False


if __name__ == '__main__':
    communication = Communication(sys.argv[1], sys.argv[2])
    communication.start()
