#!/usr/bin/env python3
# coding: utf-8

import os
import time
import rclpy
import math
from rclpy.node        import Node
from sensor_msgs.msg   import Joy, JointState, Imu, BatteryState, MagneticField
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg      import Odometry 
from .ddsm115 import DDSM115, DDSM115_STATUS


# ajirobo base
class ros2_ajirobo_base(Node):
    # constructor
    def __init__(self):
        # class

        # create node
        super().__init__('ros2_ajirobo_base')

        # parameters
        self.declare_parameter( 'device_name',  '/dev/ttyUSB0' )
        self.declare_parameter( 'wheel_radius', 0.05 )
        self.declare_parameter( 'wheel_tread',  0.34 )
        self.declare_parameter( 'left_motor_id',  1  )
        self.declare_parameter( 'right_motor_id', 2  )
        self._device_name  = self.get_parameter('device_name').value
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._wheel_tread  = float(self.get_parameter('wheel_tread').value)
        self._left_id      = int(self.get_parameter('left_motor_id').value)
        self._right_id     = int(self.get_parameter('right_motor_id').value)        

        # publisher
        #self._pub_state_joy      = self.create_publisher( Joy,           'states/joy',          10 )
        #self._pub_state_joint    = self.create_publisher( JointState,    'states/jointState',   10 )
        #self._pub_state_imu      = self.create_publisher( Imu,           'states/imu',          10 )
        #self._pub_state_mag      = self.create_publisher( MagneticField, 'states/mag',          10 )
        #self._pub_state_battery  = self.create_publisher( BatteryState,  'states/batteryState', 10 )
        self._pub_state_odom      = self.create_publisher( Odometry,      'states/odom',         10 ) 
        # subscriber
        self._sub_controller_cmdvel = self.create_subscription( Twist, '/cmd_vel', self._callback_cmdvel, 10 )

        # variable
        self._seq = 0
        
        # initialize motor
        self._motor = DDSM115( device=self._device_name, ids=[self._left_id, self._right_id] )
        # velocity control
        self._motor.set_mode( self._left_id,  2 )
        self._motor.set_mode( self._right_id, 2 )
        # enable brake
        self._motor.set_brake( self._left_id  )
        self._motor.set_brake( self._right_id )
        # stop motor
        self._motor.send_velocity( self._left_id , 0 )
        self._motor.send_velocity( self._right_id, 0 )
        
        self._l_stat = DDSM115_STATUS( id=self._left_id,  current=0.0, velocity=0.0, position=0.0, error=0.0 )
        self._r_stat = DDSM115_STATUS( id=self._right_id, current=0.0, velocity=0.0, position=0.0, error=0.0 )

        # 位置と角度の初期化
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        # 前回の時間
        self._last_time = self.get_clock().now()

        # 定期的にオドメトリを更新
        self._timer = self.create_timer(0.1, self.update_odometry)  # 0.1秒間隔

    # destructor
    def __del__(self):
        return


    # joy callback
    def _callback_joy(self, msg):
        return


    # cmd vel callback
    def _callback_cmdvel(self, msg):
        m_s   = msg.linear.x
        rad_s = msg.angular.z
        # calc motor speed
        l_vel = ( m_s+(self._wheel_tread/2.0)*rad_s ) * 60.0 / (2.0*3.141592*self._wheel_radius)
        r_vel = ( m_s-(self._wheel_tread/2.0)*rad_s ) * 60.0 / (2.0*3.141592*self._wheel_radius)
        # set speed
        self._l_stat = self._motor.send_velocity( self._left_id,  int( -l_vel ) )
        self._r_stat = self._motor.send_velocity( self._right_id, int(  r_vel ) )    
        return


    # report joy
    def report_joy(self):
        _joy = Joy()
        # header
        _joy.header.seq      = self._seq
        _joy.header.stamp    = rospy.Time.Now()
        _joy.header.frame_id = 'ajirobo'
        # axis
        _joy.axes    = [0.0, 0.0, 0.0, 0.0]
        # buttons
        _joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # publish
        self._pub_state_joy.publish(_joy)


    # report joint state
    def report_joint_state(self):
        _js = JointState()
        # header
        _js.header.seq      = self._seq
        _js.header.stamp    = rclpy.Time.Now()
        _js.header.frame_id = 'ajirobo'
        # state
        _js.name     = [ 'leftWheel', 'rightWheel' ]
        _js.position = [ 0, 0 ]
        _js.velocity = [ 0, 0 ]
        _js.effort   = [ 0, 0 ]
        # publish
        self._pub_state_joint.publish(_js)


    # report IMU data
    def report_imu(self):
        _imu = Imu()
        _mag = MagneticField()
        # header
        _imu.header.seq      = self._seq
        _imu.header.stamp    = rospy.Time.Now()
        _imu.header.frame_id = 'ajirobo'
        # orientation
        _imu.orientation.x = 0.0
        _imu.orientation.y = 0.0
        _imu.orientation.z = 0.0
        _imu.orientation.w = 1.0
        _imu.orientation_covariance = [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ]
        # angular velocity
        _imu.angular_velocity.x = 0.0
        _imu.angular_velocity.y = 0.0
        _imu.angular_velocity.z = 0.0
        _imu.angular_velocity_covariance = [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ]
        # linear acceleration
        _imu.linear_acceleration.x = 0.0
        _imu.linear_acceleration.y = 0.0
        _imu.linear_acceleration.z = 0.0
        _imu.linear_acceleration_covariance = [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ]
        # publish
        self._pub_state_imu.publish(_imu)

        # header
        _mag.header.seq      = self._seq
        _mag.header.stamp    = rospy.Time.Now()
        _mag.header.frame_id = 'ajirobo'
        # magnetic field
        _mag.magnetic_field.x = 0.0
        _mag.magnetic_field.y = 0.0
        _mag.magnetic_field.z = 0.0
        _mag.magnetic_field_covariance = [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ]
        # publish
        self._pub_state_mag.publish(_mag)


    # report battery state
    def report_battery_state(self):
        _bat = BatteryState()
        # header
        _bat.header.seq      = self._seq
        _bat.header.stamp    = rospy.Time.Now()
        _bat.header.frame_id = 'ajirobo'
        # basic
        _bat.voltage     = 18.0
        _bat.temperature = 25.0
        _bat.current     = 0.0
        # charge
        _bat.charge              = float('NaN')
        _bat.capacity            = float('NaN')
        _bat.design_capacity     = float('NaN')
        _bat.percentage          = float('NaN')
        # status
        _bat.power_supply_status     = 0
        _bat.power_supply_health     = 0
        _bat.power_supply_technology = 0
        _bat.present                 = False
        # cell
        _bat.cell_voltage     = [ float('NaN') ]
        _bat.cell_temperature = [ float('NaN') ]
        # serial
        _bat.location      = ''
        _bat.serial_number = ''
        # publish
        self._pub_state_battery.publish(_bat)

# オドメトリを更新する関数
    def update_odometry(self):
        # 現在の時間を取得
        current_time = self.get_clock().now()
        dt = (current_time - self._last_time).nanoseconds * 1e-9  # 秒に変換
        self._last_time = current_time

        # 左右のモーターのフィードバックを取得
        left_feedback = self._motor.get_motor_feedback(self._left_id)
        right_feedback = self._motor.get_motor_feedback(self._right_id)

        # フィードバックが取得できなかった場合の対策
        if left_feedback:
            self._left_rpm, self._left_current = left_feedback
        else:
            print_warning("Left motor feedback not received, setting rpm and current to 0")
            self._left_rpm, self._left_current = 0, 0.0

        if right_feedback:
            self._right_rpm, self._right_current = right_feedback
        else:
            print_warning("Right motor feedback not received, setting rpm and current to 0")
            self._right_rpm, self._right_current = 0, 0.0

        # RPMを m/s に変換
        left_velocity_mps = (self._left_rpm * 2 * math.pi * self._wheel_radius) / 60
        right_velocity_mps = (self._right_rpm * 2 * math.pi * self._wheel_radius) / 60

        # ロボットの直線速度と角速度を計算
        vx = (left_velocity_mps + right_velocity_mps) / 2.0
        vtheta = (right_velocity_mps - left_velocity_mps) / self._wheel_tread

        # ロボットの位置と向きの更新
        self._x += vx * math.cos(self._theta) * dt
        self._y += vx * math.sin(self._theta) * dt
        self._theta += vtheta * dt

        # オドメトリメッセージを作成してパブリッシュ
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # 位置と角度をセット
        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.create_quaternion_from_yaw(self._theta)

        # 速度をセット
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vtheta

        # パブリッシュ
        self._pub_state_odom.publish(odom_msg)

    # yawをクォータニオンに変換
    def create_quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q    


# main function
def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # initialize node
    ajirobo = ros2_ajirobo_base()
    rclpy.spin(ajirobo)

    # Destroy the node explicitly
    ajirobo.destroy_node()
    rclpy.shutdown()




# main function
if( __name__ == '__main__' ):
    main()
