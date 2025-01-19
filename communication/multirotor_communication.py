# 这个程序的主要功能包括：
# 1.坐标系支持：
#   .ENU(East-North-Up)坐标系
#   .FLU(Forward-Left-Up)坐标系
# 2,控制模式：
#   .位置控制
#   .速度控制
#   .加速度控制
#   .悬停控制
# 3.ROS通信：
#   .订阅多个命令话题
#   .发布目标运动控制消息
#   .提供解锁/上锁服务
#   .提供飞行模式切换服务
# 4. 状态管理：
#   .自动悬停状态切换
#   .飞行模式管理
#   .解锁状态管理
# 5.安全特性：
#   .运动指令死区检测
#   .命令重复检测
#   .错误状态反馈
# 这个程序是一个典型的ROS控制节点，用于桥接高层控制命令和低层飞行器控制接口(MAVROS)。
# 它提供了完整的三维空间运动控制能力，并包含了必要的安全保护机制。

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
import sys

# 初始化ROS节点,节点名称由传入的参数组成(飞行器类型_ID_communication)
rospy.init_node(sys.argv[1]+'_'+sys.argv[2]+"_communication")
rate = rospy.Rate(30)  # 设置循环频率为30Hz

class Communication:
    """多旋翼飞行器通信控制类
    
    用于处理飞行器的位置控制、速度控制、加速度控制等功能,
    支持ENU(东北天)和FLU(前左上)两种坐标系
    """

    def __init__(self, vehicle_type, vehicle_id):
        """初始化通信类
        
        Args:
            vehicle_type: 飞行器类型
            vehicle_id: 飞行器ID
        """
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.current_position = None
        self.current_yaw = 0
        self.hover_flag = 0
        self.coordinate_frame = 1
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback,queue_size=3)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback,queue_size=1)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_flu", Twist, self.cmd_accel_flu_callback,queue_size=1)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_enu", Twist, self.cmd_accel_enu_callback,queue_size=1)
            
        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)

        print(self.vehicle_type+'_'+self.vehicle_id+": "+"communication initialized")

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

    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+": "+msg.data)

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)  
 
    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)    

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        """构造目标运动控制消息
        
        Args:
            x,y,z: 位置控制目标值
            vx,vy,vz: 速度控制目标值
            afx,afy,afz: 加速度控制目标值
            yaw: 偏航角目标值
            yaw_rate: 偏航角速率目标值
            
        Returns:
            PositionTarget: 目标运动控制消息
        """
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        # 设置位置目标
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        # 设置速度目标
        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        # 设置加速度目标
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        # 设置姿态目标
        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        # 根据运动类型设置type_mask
        # motion_type = 0: 位置控制模式
        # motion_type = 1: 速度控制模式
        # motion_type = 2: 加速度控制模式
        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def hover_state_transition(self,x,y,z,w):
        """处理悬停状态转换
        
        当速度/加速度命令接近零时切换到悬停模式
        
        Args:
            x,y,z: 线速度/加速度
            w: 角速度
        """
        # 当运动指令大于阈值时退出悬停
        if abs(x) > 0.02 or abs(y) > 0.02 or abs(z) > 0.02 or abs(w) > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        # 当运动指令小于阈值且不在悬停模式时进入悬停
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()

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
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed!")
            return False

    def hover(self):
        """悬停控制
        
        保持当前位置和偏航角进行悬停
        """
        self.coordinate_frame = 1  # 使用ENU坐标系
        self.motion_type = 0      # 使用位置控制
        # 将当前位置设为目标位置
        self.target_motion = self.construct_target(
            x=self.current_position.x,
            y=self.current_position.y,
            z=self.current_position.z,
            yaw=self.current_yaw
        )
        print(self.vehicle_type+'_'+self.vehicle_id+":"+self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        elif self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()
