#!/usr/bin/env python2
# encoding: utf-8
# LiDAR obstacle avoidance and lidar following function(雷达避障，跟随功能)
import os
import math
import time
import rclpy
import threading
import numpy as np
import pandas as pd
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from interfaces.srv import SetInt64, SetFloat64List
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros_robot_controller_msgs.msg import MotorState, SetPWMServoState, PWMServoState
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

CAR_WIDTH = 0.4  # meter
MAX_SCAN_ANGLE = 240


class LidarController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.running_mode = 0
        self.threshold = 0.6  # meters
        self.scan_angle = math.radians(360)  # radians
        self.speed = 0.2
        self.last_act = 0
        self.timestamp = 0
        self.angle_data = []
        # pid parameter(pid参数)
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # Chassis control(底盘控制)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # Enter the feature(进入玩法)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # Exit the feature(退出玩法)
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)  # Start the feature(开启玩法)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.heartbeat = Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response())) # Heartbeat package(心跳包)
        self.create_service(SetFloat64List, '~/set_param', self.set_parameters_srv_callback)  # Set parameter(参数设置)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def reset_value(self):
        '''
        Reset parameter(重置参数)
        :return:
        '''
        self.running_mode = 0
        self.threshold = 0.6
        self.scan_angle = math.radians(90)
        self.speed = 0.2
        self.last_act = 0
        self.speed = 0.2
        self.timestamp = 0
        self.pid_yaw.clear()
        self.pid_dist.clear()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar enter')
        if hasattr(self, 'heartbeat') and self.heartbeat:
            self.heartbeat.start()
        self.reset_value()
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)  # Subscribe to Lidar data(订阅雷达数据)
        set_servo_position(self.joints_pub, 1, ((10, 500), (5, 500), (4, 180), (3, 50), (2, 700), (1, 500)))
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar exit')
        if hasattr(self, 'heartbeat') and self.heartbeat:
            self.heartbeat.stop()
        self.reset_value()
        self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        '''
        Enable different functions(开启不同功能)
        :param req: 0 close, 1 obstacle avoidance, 2 LiDAR following, 3 LiDAR guarding(0关闭，1避障，2跟随，3警卫)
        :return:
        '''
        self.get_logger().info(str(request))
        new_running_mode = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % ("set_running " + str(new_running_mode)))
        if not 0 <= new_running_mode <= 3:
            response.success = False
            response.message = "Invalid running mode {}".format(new_running_mode)
        else:
            response.success = True
            response.message = "set_running"
            with self.lock:
                self.running_mode = new_running_mode
        self.mecanum_pub.publish(Twist())
        return response

    def set_parameters_srv_callback(self, request, response):
        '''
        Set the threshold of obstacle avoidance and speed(设置避障阈值，速度参数)
        :param req:
        :return:
        '''
        new_parameters = request.data
        new_threshold, new_scan_angle, new_speed = new_parameters
        self.get_logger().info("\033[1;32mn_t:{:2f}, n_a:{:2f}, n_s:{:2f}\033[0m".format(new_threshold, new_scan_angle, new_speed))
        if not 0.3 <= new_threshold <= 1.5:
            response.success = False
            response.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
            return response
        if not 0 <= new_scan_angle <= 90:
            response.success = False
            response.message = "New scan angle ({:.2f}) is out of range (0 ~ 90)"
            return response
        if not new_speed > 0:
            response.success = False
            response.message = "Invalid speed"
            return response

        with self.lock:
            self.threshold = new_threshold
            self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed
            self.speed = self.speed
        return response

    def lidar_callback(self, lidar_data):
        # LiDAR subscription callback(雷达订阅回调)
        twist = Twist()
        # Data size = scanning angle / the increased angle per scan(数据大小 = 扫描角度/每扫描一次增加的角度)
        if self.lidar_type != 'G4':
            if self.lidar_type == 'SCLIDAR':
                num_points = len(lidar_data.ranges)
                shifted_ranges = list(lidar_data.ranges[num_points//2:]) + list(lidar_data.ranges[:num_points//2])
                max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
                left_ranges = shifted_ranges[:max_index]
                right_ranges = shifted_ranges[::-1][:max_index]
            else:
                max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
                left_ranges = lidar_data.ranges[:max_index]  # The left data (左半边数据)
                right_ranges = lidar_data.ranges[::-1][:max_index]  # The right data (右半边数据)
        elif self.lidar_type == 'G4':
            '''
                ranges[right...->left]

                    forward
                    lidar
                left 0 right

            '''
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = min_index + int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # The left data (左半边数据)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # The right data (右半边数据)
            # self.get_logger().info(str(left_ranges))


        with self.lock:
            # Get the data according to the settings(根据设定取数据)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])
            # if self.machine_type != 'MentorPi_Acker' or self.machine_type != 'MentorPi_Acker_Pro':
            if 'Acker' not in self.machine_type:
                if self.running_mode == 1 and self.timestamp <= time.time():
                    left_nonzero = left_range.nonzero()
                    right_nonzero = right_range.nonzero()
                    left_nonan = np.isfinite(left_range[left_nonzero])
                    right_nonan = np.isfinite(right_range[right_nonzero])
                    min_dist_left_ = left_range[left_nonzero][left_nonan]
                    min_dist_right_ = right_range[right_nonzero][right_nonan]
                    # Take the nearest distance left and right(取左右最近的距离)
                    if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                        min_dist_left = min_dist_left_.min()
                        min_dist_right = min_dist_right_.min()
                        if min_dist_left <= self.threshold and min_dist_right > self.threshold:  # There is obstacle at left(左侧有障碍)
                            twist.linear.x = self.speed / 6  # keep the forward velocity as low as possible when turning(转弯时向前速度尽量小)
                            max_angle = math.radians(90)  # Turn 90 degrees(转90度)
                            w = self.speed * 6.0  # keep the turning speed as high as possible(转弯速度尽量大)
                            twist.angular.z = -w
                            if self.last_act != 0 and self.last_act != 1:
                                twist.angular.z = w
                            self.last_act = 1
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # There are obstacles on both sides(两侧都有障碍)
                            twist.linear.x = self.speed / 6  # keep the forward velocity as low as possible when turning(转弯时向前速度尽量小)
                            w = self.speed * 6.0  # keep the turning speed as high as possible(转弯速度尽量大)
                            twist.angular.z = w
                            self.last_act = 3
                            self.mecanum_pub.publish(twist)
                            # Turn 180 degrees(转180度)
                            self.timestamp = time.time() + (math.radians(180) / w / 2)
                        elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # There is obstacles at right(右侧有障碍)
                            twist.linear.x = self.speed / 6  # keep the forward velocity as low as possible when turning(转弯时向前速度尽量小)
                            max_angle = math.radians(90)  # Turn 90 degrees(转90度)
                            w = self.speed * 6.0  # keep the turning speed as high as possible(转弯速度尽量大)
                            twist.angular.z = w
                            if self.last_act != 0 and self.last_act != 2:
                                twist.angular.z = -w
                            self.last_act = 2
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        else:  # None obstacle(没有障碍)
                            self.last_act = 0
                            twist.linear.x = self.speed  # Go straight(直行)
                            self.mecanum_pub.publish(twist)
                elif self.running_mode == 2:
                    # The merged distance data from right half counterclockwise to the left half(拼合距离数据, 从右半侧逆时针到左半侧)
                    ranges = np.append(right_range[::-1], left_range)
                    self.get_logger().info(str(ranges))
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 0:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        angle = -angle + lidar_data.angle_increment * min_index  # Calculate the angle corresponding to the minimum value(计算最小值对应的角度)
                        # self.get_logger().info(str(dist_))
                        # self.get_logger().info(str([dist, angle]))
                        if dist < self.threshold and abs(math.degrees(angle)) > 5:  # Control the left and right(控制左右)
                            if self.lidar_type != 'G4':
                                self.pid_yaw.update(-angle)
                                twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0) 
                            elif self.lidar_type == 'G4':
                                self.pid_yaw.update(angle)
                                twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0) 
                        else:
                            self.pid_yaw.clear()

                        if dist < self.threshold and abs(0.2 - dist) > 0.02:  # Control the front and back(控制前后)
                            self.pid_dist.update(self.threshold / 2 - dist)
                            twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                        else:
                            self.pid_dist.clear()
                        if abs(twist.angular.z) < 0.008:
                            twist.angular.z = 0.0
                        if abs(twist.linear.x) < 0.05:
                            twist.linear.x = 0.0
                        self.mecanum_pub.publish(twist)
                elif self.running_mode == 3:
                    # The merged distance data from right half counterclockwise to the left half(拼合距离数据, 从右半侧逆时针到左半侧)
                    ranges = np.append(right_range[::-1], left_range)
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 1:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        
                        angle = -angle + lidar_data.angle_increment * min_index  # Calculate the angle corresponding to the minimum value(计算最小值对应的角度)
                        if dist < self.threshold and abs(math.degrees(angle)) > 5:  # Control the left and right(控制左右)
                            if self.lidar_type != 'G4':
                                self.pid_yaw.update(-angle)
                                twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                            elif self.lidar_type == 'G4':
                                self.pid_yaw.update(angle)
                                twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0) 
                        else:
                            self.pid_yaw.clear()
                        if abs(twist.angular.z) < 0.008:
                            twist.angular.z = 0.0
                        self.mecanum_pub.publish(twist)
            else:
                if self.running_mode == 1 and self.timestamp <= time.time():
                    left_nonzero = left_range.nonzero()
                    right_nonzero = right_range.nonzero()
                    left_nonan = np.isfinite(left_range[left_nonzero])
                    right_nonan = np.isfinite(right_range[right_nonzero])
                    # Take the nearest distance left and right(取左右最近的距离)
                    min_dist_left_ = left_range[left_nonzero][left_nonan]
                    min_dist_right_ = right_range[right_nonzero][right_nonan]
                    if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                        min_dist_left = min_dist_left_.min()
                        min_dist_right = min_dist_right_.min()
                        if self.last_act == 3: 
                            twist.linear.x = self.speed
                            twist.angular.z = -twist.linear.x/0.7 # # Set right turn radius to 0.1m(右转转弯半径设为0.1m)
                            self.last_act = 4
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + 0.5
                        elif self.last_act == 4:
                            self.last_act = 5
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + 0.1
                        elif (min_dist_left <= self.threshold and min_dist_right > self.threshold):  # There is obstacle at left(左侧有障碍)
                            twist.linear.x = self.speed
                            twist.angular.z = -twist.linear.x/0.7 # Turn right(右转)
                            self.last_act = 1
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + 1
                        elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # There are obstacles on both sides(两侧都有障碍)
                            twist.linear.x = -0.3
                            servo_state = PWMServoState()
                            servo_state.id = [1] 
                            servo_state.position = [1500] 
                            data = SetPWMServoState()
                            data.state = [servo_state]
                            data.duration = 0.02
                            self.servo_state_pub.publish(data)
                            self.last_act = 3
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + 1
                            self.last_act = 3
                            self.mecanum_pub.publish(twist)
                            self.timestamp = time.time() + 1
                        elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # There is obstacles at right(右侧有障碍)
                            if self.last_act == 5:
                                twist.linear.x = -0.3  # Move backward(后退)
                                twist.angular.z = -twist.linear.x/0.7 
                                self.last_act = 3
                                self.mecanum_pub.publish(twist)
                                self.timestamp = time.time() + 1
                            else:
                                twist.linear.x = self.speed
                                twist.angular.z = -twist.linear.x/0.7
                                self.last_act = 2
                                self.mecanum_pub.publish(twist)
                                self.timestamp = time.time() + 1
                        else:
                             # When there is no obstacle, go straight and ensure the servo is centered(没有障碍物时，直行并确保舵机回中)
                            self.last_act = 0
                            twist.linear.x = 0.3

                            # Add servo centering command(添加舵机回中指令)
                            servo_state = PWMServoState()
                            servo_state.id = [1]  # please replace [1] with the actual servo ID(请将 [1] 替换为您实际使用的舵机ID)
                            servo_state.position = [1500]  # Central position(中间位置)
                            data = SetPWMServoState()
                            data.state = [servo_state]
                            data.duration = 0.02  # Action duration, adjust as needed(动作持续时间，可根据需要调整)
                            self.servo_state_pub.publish(data)

                            self.pid_yaw.clear()
                            self.mecanum_pub.publish(twist)

                elif self.running_mode == 2:
                    # The merged distance data from right half counterclockwise to the left half(拼合距离数据, 从右半侧逆时针到左半侧)
                    ranges = np.append(right_range[::-1], left_range)
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 1:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        angle = -angle + lidar_data.angle_increment * min_index  # Calculate the angle corresponding to the minimum value(计算最小值对应的角度)
                        self.angle_data.append(angle)
                        data = pd.DataFrame(self.angle_data)
                        data_ = data.copy()
                        u = data_.mean()  # Calculate the mean(计算均值)
                        std = data_.std()  # Calculate the standard deviation(计算标准差)

                        data_c = data[np.abs(data - u) <= std]
                        data_c = data_c.dropna(axis=0, how='any')
                        angle = data.tail(1).iloc[0, 0]
                        if len(self.angle_data) == 15:
                            self.angle_data.remove(self.angle_data[0])
                        if dist < 2 and abs(0.5 - dist) > 0.02:  # Control the front and back(控制前后)
                            self.pid_dist.SetPoint = 0.5
                            self.pid_dist.update(dist)
                            twist.linear.x = common.set_range(-self.pid_dist.output, -0.35, 0.35)
                        else:
                            self.pid_dist.clear()
                        if dist < 2 and abs(math.degrees(angle)) > 5:  # Control the left and right(控制左右)
                            if self.lidar_type != 'G4':
                                self.pid_yaw.update(-angle)
                                twist.angular.z = twist.linear.x*math.tan(common.set_range(self.pid_yaw.output, -0.316, 0.316))/0.145
                            elif self.lidar_type == 'G4':
                                self.pid_yaw.update(angle)
                                twist.angular.z = -twist.linear.x*math.tan(common.set_range(self.pid_yaw.output, -0.316, 0.316))/0.145
                        else:
                            self.pid_yaw.clear()
                        self.mecanum_pub.publish(twist)

def main():
    node = LidarController('lidar_app')
    rclpy.spin(node)  # Wait in a loop for ROS2 to exit(循环等待ROS2退出)

if __name__ == "__main__":
    main()

